#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>

int main(void) {
    struct stat statbuf;
	int in_fd = open("raw.bin", O_RDONLY);
	FILE *output = fopen("decoded.wav", "w+b");

    if (-1 == fstat(in_fd, &statbuf)) {
        perror("unable to determine size of buffer");
        return 1;
    }

    static uint8_t wav_header[44] = {
        0x52, 0x49, 0x46, 0x46, 0x1c, 0x12, 0x05, 0x00, 0x57, 0x41, 0x56,
        0x45, 0x66, 0x6d, 0x74, 0x20, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00,
        0x01, 0x00, 0x11, 0x2b, 0x00, 0x00, 0x22, 0x56, 0x00, 0x00, 0x02,
        0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0xf8, 0x11, 0x05, 0x00,
    };
    uint32_t rate = 62500;
    memcpy(&wav_header[24], &rate, sizeof(rate));
    *((uint32_t *)&(wav_header[40])) = statbuf.st_size * 16;

    if (fwrite(wav_header, sizeof(wav_header), 1, output) <= 0) {
        perror("error");
        fclose(output);
        return 1;
    }

    int baselen = statbuf.st_size - sizeof(wav_header);
    uint8_t *raw_buffer = malloc(baselen * sizeof(int16_t));
    if ((unsigned int)read(in_fd, raw_buffer, baselen) !=
        statbuf.st_size - sizeof(wav_header)) {
        fprintf(stderr, "short read of raw file\n");
        return 1;
    }

	int16_t high = 16384;
	int16_t low = -16384;
	int off;
	for (off = 0; off < baselen; off++) {
		uint8_t c = raw_buffer[off];
		int bit = 0;
		for (bit = 0; bit < 8; bit++) {
			if (c & (1 << bit)) {
				fwrite(&high, sizeof(high), 1, output);
			} else {
				fwrite(&low, sizeof(low), 1, output);
			}
		}
	}
	fclose(output);

	return 0;
}
