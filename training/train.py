import sys
sys.dont_write_bytecode = True   # 不生成 __pycache__：model/、utils/ 保持干净
import os
import math
import argparse
from tqdm import tqdm

import torch
from torch import optim

from torchsummary import summary

import utils.loss
import utils.utils
import utils.datasets
import model.detector

# 工作目录固定到本文件所在的 training/：脚本内部（以及命令行传入的）相对路径都按它解析，
# 所以从任何目录启动结果都一样。见交付文档 2.2 节。
os.chdir(os.path.dirname(os.path.abspath(__file__)))


if __name__ == '__main__':
    # 指定训练配置文件
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str, default='data/voc.data',
                        help='Specify training profile *.data')
    opt = parser.parse_args()
    cfg = utils.utils.load_datafile(opt.data)

    print("训练配置:")
    print(cfg)

    # 数据集加载
    train_dataset = utils.datasets.TensorDataset(cfg["train"], cfg["width"], cfg["height"], imgaug = True)
    val_dataset = utils.datasets.TensorDataset(cfg["val"], cfg["width"], cfg["height"], imgaug = False)

    batch_size = int(cfg["batch_size"] / cfg["subdivisions"])
    nw = min([os.cpu_count(), batch_size if batch_size > 1 else 0, 8])
    # 训练集
    train_dataloader = torch.utils.data.DataLoader(train_dataset,
                                                   batch_size=batch_size,
                                                   shuffle=True,
                                                   collate_fn=utils.datasets.collate_fn,
                                                   num_workers=0, #num_workers=nw,
                                                   pin_memory=False, #pin_memory=True,
                                                   drop_last=True,
                                                   persistent_workers=False  #persistent_workers=True
                                                   )
    #验证集
    val_dataloader = torch.utils.data.DataLoader(val_dataset,
                                                 batch_size=batch_size,
                                                 shuffle=False,
                                                 collate_fn=utils.datasets.collate_fn,
                                                 num_workers=0,#num_workers=nw,
                                                 pin_memory=False,#pin_memory=True,
                                                 drop_last=False,
                                                 persistent_workers=False  #persistent_workers=True
                                                 )

    # 指定后端设备CUDA&CPU
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # 分类损失的类别权重：出厂权重有 18 类在验证集上 AP 恰好为 0（模型根本不发这些字符），
    # 是长尾造成的 —— 最少的类只有 5 个框，最多的 2000 个，差 400 倍。
    # class_weights() 只放大稀有类，常见类（含 plate 自己）权重恒为 1.0，不改损失尺度。
    cls_weights, cls_counts = utils.utils.class_weights(cfg["train"], cfg["classes"])
    if cls_weights is None:
        print("警告：算不出类别权重（读不到 %s 的标注），本次用无权重 CE" % cfg["train"])
        cls_weight = None
    else:
        cls_weight = torch.tensor(cls_weights, dtype = torch.float32, device = device)
        names = utils.utils.load_names(cfg["names"])
        # 只打抬得最狠的 12 类够核对了；70 类的完整 AP 明细在下面 evaluation 的 verbose 里
        print("分类损失类别权重（只上抬稀有类，1.0 = 与改动前相同）:")
        for i in sorted(range(cfg["classes"]), key = lambda c: -cls_weights[c])[:12]:
            print("   %2d %-6s n=%-6d w=%.3f" %
                  (i, names[i] if names else "?", cls_counts[i], cls_weights[i]))

    # 判断是否加载预训练模型
    load_param = False
    premodel_path = cfg["pre_weights"]
    if premodel_path != None and os.path.exists(premodel_path):
        load_param = True

    # 初始化模型结构
    model = model.detector.Detector(cfg["classes"], cfg["anchor_num"], load_param).to(device)
    summary(model, input_size=(3, cfg["height"], cfg["width"]))

    # 加载预训练模型参数
    if load_param == True:
        model.load_state_dict(torch.load(premodel_path, map_location=device), strict = False)
        print("Load finefune model param: %s" % premodel_path)
    else:
        print("Initialize weights: model/backbone/backbone.pth")

    # 构建SGD优化器
    optimizer = optim.SGD(params=model.parameters(),
                          lr=cfg["learning_rate"],
                          momentum=0.949,
                          weight_decay=0.0005,
                          )

    # 学习率衰减策略
    scheduler = optim.lr_scheduler.MultiStepLR(optimizer,
                                               milestones=cfg["steps"],
                                               gamma=0.1)

    print('Starting training for %g epochs...' % cfg["epochs"])

    batch_num = 0
    for epoch in range(cfg["epochs"]):
        model.train()
        pbar = tqdm(train_dataloader)

        for imgs, targets in pbar:
            # 数据预处理
            imgs = imgs.to(device).float() / 255.0
            targets = targets.to(device)

            # 模型推理
            preds = model(imgs)
            # loss计算
            iou_loss, obj_loss, cls_loss, total_loss = utils.loss.compute_loss(preds, targets, cfg, device, cls_weight)

            # 反向传播求解梯度
            total_loss.backward()

            #学习率预热
            for g in optimizer.param_groups:
                warmup_num =  5 * len(train_dataloader)
                if batch_num <= warmup_num:
                    scale = math.pow(batch_num/warmup_num, 4)
                    g['lr'] = cfg["learning_rate"] * scale
                    
                lr = g["lr"]

            # 更新模型参数
            if batch_num % cfg["subdivisions"] == 0:
                optimizer.step()
                optimizer.zero_grad()

            # 打印相关信息
            info = "Epoch:%d LR:%f CIou:%f Obj:%f Cls:%f Total:%f" % (
                    epoch, lr, iou_loss, obj_loss, cls_loss, total_loss)
            pbar.set_description(info)

            batch_num += 1

        # 模型保存：每 10 轮一次，外加最后一轮（range(epochs) 取到 epochs-1，否则 epochs=300 时最后一刀停在 290）
        if (epoch % 10 == 0 or epoch == cfg["epochs"] - 1) and epoch > 0:  #if (epoch % 10 == 0 or epoch == cfg["epochs"] - 1) and epoch > 0:
            model.eval()
            #模型评估
            print("computer mAP...")
            # verbose=True：连逐类明细一起打（70 行），用来判断 0.2x 的 mAP 里有多少是长尾省份拖的
            _, _, AP, _ = utils.utils.evaluation(val_dataloader, cfg, model, device, verbose = True)
            print("computer PR...")
            precision, recall, _, f1 = utils.utils.evaluation(val_dataloader, cfg, model, device, 0.3)
            print("Precision:%f Recall:%f AP:%f F1:%f"%(precision, recall, AP, f1))

            torch.save(model.state_dict(), "weights/%s-%d-epoch-%fap-model.pth" %
                      (cfg["model_name"], epoch, AP))

        # 学习率调整
        scheduler.step()
