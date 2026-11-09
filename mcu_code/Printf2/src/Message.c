#include "Message.h"
/* 160 AND NOT 64, because 64 was already too small for what the diagnostics
 * print into it - measured, not estimated.  The real capture kept in
 * tools/_recon_fix.txt carries
 *     gpu cv chk n47 cin72 cout3 K3 10x10 px300 lim12 worst 44 bad 94 FALLBACK
 * which is 76 characters, and `gpu cv loc` is longer still on a wide layer.
 * Both go through sprintf() with no bound, so both were writing past the end of
 * this array.  The object that follows it in Printf.map is USART_InitStructure
 * (0x30000938 + 64 = 0x30000978, size 28), so the overflow landed in THAT
 * struct - it is written once at boot (main.c:229-236) and never read again,
 * which is the only reason the corruption was never seen.
 *
 * The lines looked perfect either way.  sprintf() writes its terminator beyond
 * the array, so UART_SendString() stops exactly where the text ends and the
 * damage is invisible from the terminal - the same shape as the static-
 * initialiser and Message-length traps this project has already paid for twice.
 *
 * 160 covered the longest line the firmware could build at the time (the `chk`
 * line with its rail/at0 columns is about 121 characters at the widest layer),
 * and 96 extra bytes of .bss is nothing beside the 2 MB arena.  Nothing reads
 * this array's size - there is no sizeof(Message) anywhere in the tree - so the
 * only consequence of the change is that every symbol after it in .bss moves.
 *
 * 2026-10-20: 192, because 160 was too small AGAIN.  The cascade line grew a
 * `why %d n2 %d` tail and hit 164 characters on the widest round - which put
 * the terminator 5 bytes into USART_InitStructure.  Same shape as before: the
 * line still looked perfect on the terminal, so nothing pointed at it.  If this
 * line ever grows a field again, measure the worst case instead of guessing:
 * every %d here can be 10 characters plus a sign. */
char Message[192];

/* 只有 DISPLAY_PLATE_ONLY=1 才有人读它 (见 Message.h 的说明)。 */
volatile uint8_t Message_allow = 0;

void UART_SendString(const char* str) {
#if DISPLAY_PLATE_ONLY
    /* 三行开机标记按**前缀**放行 —— 调用点完全不用知道有这个开关。
       只比前 N 个字符, 因为 `yolo start ...` 后面还可能跟东西。
       `yolo init ERROR` 一并放行 (2026-10-20 用户要求): 它后面紧跟 while(1),
       被拦掉的话板子会停在"打了 yolo start 就再没动静"的状态, 而这正好是
       "没起来"和"起来了没识别到"最难分的那一档。 失败行必须能看见。 */
    if (!Message_allow) {
        const int is_start = (strncmp(str, "yolo start", 10) == 0);
        const int is_init  = (strncmp(str, "yolo init OK", 12) == 0);
        const int is_err   = (strncmp(str, "yolo init ERROR", 15) == 0);
        if (!is_start && !is_init && !is_err) return;
    }
#endif
    while (*str) {
			while (USART_GetFlagStatus(USARTx, USART_FLAG_TXC) == RESET)
			{
				//wait send transmission completed
			}
      USART_SendData(USARTx, (uint32_t)*str++);
    }
}


