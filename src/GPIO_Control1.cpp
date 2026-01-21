#include <iostream>
#include <stdint.h>
#include <unistd.h>
#include <sys/io.h>

using UINT16 = unsigned short;
using UINT32 = unsigned long;

#define PCI_INDEX 0xcf8
#define PCI_DATA  0xcfc

static inline void delay_ms(int ms) { usleep(ms * 1000); }

static inline UINT32 ReadPciDword(uint8_t bus, UINT16 dev, UINT16 fun, UINT16 reg)
{
    outl(0x80000000u + (bus << 16) + (dev << 11) + (fun << 8) + reg, PCI_INDEX);
    return inl(PCI_DATA);
}

static UINT16 get_smbase()
{
    for (int dev = 31; dev >= 0; --dev) {
        for (int fun = 0; fun < 8; ++fun) {
            UINT32 cls = ReadPciDword(0, dev, fun, 0x08) & 0xFFFF0000;
            if (cls == 0xFFFFFFFF) continue;
            if (cls == 0x0C050000) { // SMBus Controller
                return (ReadPciDword(0, dev, fun, 0x20) & 0xFFFE);
            }
        }
    }
    return 0;
}

class Gpio9554Direct {
public:
    explicit Gpio9554Direct(uint8_t addr_8bit_shifted) : addr(addr_8bit_shifted)
    {
        // 必须 root
        if (iopl(3) != 0) {
            perror("iopl(3)");
        }
        smbase = get_smbase();
        if (!smbase) {
            std::cerr << "ERROR: SMBus controller not found (smbase=0)\n";
        }
    }

    // 设置某 bit 为输出：config(bit)=0
    bool setOutput(uint8_t bit)
    {
        uint8_t cfg = read_byte(0x03);
        cfg &= ~(1u << bit);
        return write_byte(0x03, cfg);
    }

    // 设置某 bit 为输入：config(bit)=1
    bool setInput(uint8_t bit)
    {
        uint8_t cfg = read_byte(0x03);
        cfg |= (1u << bit);
        return write_byte(0x03, cfg);
    }

    // 写输出 bit（会先保证该 bit 配置为输出）
    bool writeBit(uint8_t bit, uint8_t val)
    {
        if (val > 1) return false;

        // 先把该 bit 配成输出（和你 demo 一样）
        uint8_t cfg = read_byte(0x03);
        cfg &= ~(1u << bit);
        if (!write_byte(0x03, cfg)) return false;

        uint8_t outv = read_byte(0x01);
        outv = (outv & ~(1u << bit)) | (val << bit);
        return write_byte(0x01, outv);
    }

    // 读输入 bit
    int readBit(uint8_t bit)
    {
        uint8_t inv = read_byte(0x00);
        return (inv >> bit) & 0x1;
    }

private:
    UINT16 smbase = 0;
    uint8_t addr = 0; // 注意：这里用的是 demo 里那种“8-bit 左移地址”，如 0x72

private:
    uint8_t read_byte(uint8_t reg)
    {
        if (!smbase) return 0xFF;

        // === 完全按 demo 的节奏来 ===
        for (int i = 0; i < 100; i++) {
            outb(0xFF, smbase);      // clear status
            delay_ms(5);

            if ((inb(smbase + 0x00) & 0x01) == 0x00)
                break;

            if (i == 99) {
                std::cerr << "SMBus timeout (read)\n";
                return 0xFF;
            }
        }

        // disable PEC first
        outb(inb(smbase + 2) & 0x7F, smbase + 2);
        outb(0x00, smbase + 0x0D);

        outb(reg, smbase + 3);           // command
        outb(addr | 0x01, smbase + 4);   // slave addr + read
        outb(0x48, smbase + 2);          // byte data protocol + start
        delay_ms(5);

        if ((inb(smbase) & 0x04) == 0x04) {
            std::cerr << "SMBus access error (read)\n";
        }

        return inb(smbase + 5);
    }

    bool write_byte(uint8_t reg, uint8_t val)
    {
        if (!smbase) return false;

        for (int i = 0; i < 100; i++) {
            outb(0xFF, smbase);  // clear status
            delay_ms(5);

            if ((inb(smbase + 0x00) & 0x01) == 0x00)
                break;

            if (i == 99) {
                std::cerr << "SMBus timeout (write)\n";
                return false;
            }
        }

        delay_ms(5);
        outb(reg, smbase + 3);
        delay_ms(5);
        outb(addr & 0xFE, smbase + 4);   // slave addr + write
        delay_ms(5);
        outb(val, smbase + 5);          // data
        delay_ms(5);
        outb(0x48, smbase + 2);         // start
        delay_ms(5);

        if ((inb(smbase) & 0x04) == 0x04) {
            std::cerr << "SMBus access error (write)\n";
            return false;
        }
        return true;
    }
};

int main()
{
    // 重要：这里仍然用 0x72（demo 风格：8-bit 左移地址）
    Gpio9554Direct gpio(0x72);

    std::cout << "GPIO HIGH, measure now...\n";
    // gpio.writeBit(3, 1);         // DB9-2 (bit3) 拉高

    std::cout << "Read GPIO7...\n";
    int val = gpio.readBit(7);   // DB9-7 (bit7) 读取
    std::cout << "GPIO7 = " << val << "\n";

    std::cout << "GPIO LOW\n";
    // gpio.writeBit(3, 0);         // 拉低

    return 0;
}
