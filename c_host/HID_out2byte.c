#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <string.h>

int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <hidraw_device:/dev/hidrawX> <led_byte:hex or dec>\n", argv[0]);
        return EXIT_FAILURE;
    }

    const char *hidraw_device = argv[1];
    unsigned char led_byte = (unsigned char)strtol(argv[2], NULL, 0); // 將輸入的字串轉換為 byte
    int fd;

    // 開啟 HID 裝置
    fd = open(hidraw_device, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open HID device");
        return EXIT_FAILURE;
    }

    // 準備寫入的 buffer
    unsigned char buf[2];
    buf[0] = led_byte;
    buf[1] = 0;

    // 寫入數據到 HID 裝置
    if (write(fd, buf, sizeof(buf)) < 0) {
        perror("Failed to write to HID device");
        close(fd);
        return EXIT_FAILURE;
    }

    printf("Successfully wrote to the HID device %s: %02X\n", hidraw_device, buf[0]);

    // 關閉裝置
    close(fd);
    return EXIT_SUCCESS;
}

