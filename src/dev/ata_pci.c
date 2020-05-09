

#include <nautilus/nautilus.h>
#include <nautilus/blkdev.h>
#include <dev/ata_pci.h>

//#ifndef NAUT_CONFIG_DEBUG_ATA_PCI
#undef DEBUG_PRINT
#define DEBUG_PRINT(fmt, args...)
//#endif

#define ERROR(fmt, args...) ERROR_PRINT("ata_pci: " fmt, ##args)
#define DEBUG(fmt, args...) DEBUG_PRINT("ata_pci: " fmt, ##args)
#define INFO(fmt, args...) INFO_PRINT("ata_pci: " fmt, ##args)


#define STATE_LOCK_CONF uint8_t _state_lock_flags
#define STATE_LOCK(state) _state_lock_flags = spin_lock_irq_save(&state->lock)
#define STATE_UNLOCK(state) spin_unlock_irq_restore(&(state->lock), _state_lock_flags)

struct prdt {
	uint32_t buffer_phys;
	uint16_t transfer_size;
	uint16_t mark_end;
}__attribute__((packed));

typedef struct prdt* prdt_t;

struct ata_blkdev_state {
    struct nk_block_dev *blkdev;

    spinlock_t lock;

    prdt_t prdt;

    uint8_t* prdt_phys;

    uint8_t* mem_buffer;
    uint8_t* mem_buffer_phys; 

    struct ata_controller_state *controller;

    enum {NONE=0, HD, CD} type;
    uint64_t            block_size;
    uint64_t            num_blocks;
    uint8_t             channel; // 0/1 on controller (primary/secondary)
    uint8_t             id;      // 0/1 on channel (master/slave)
};

struct ata_controller_state {
    // devices 0,1 are master/slave on primary
    // devices 2,3 are master/slave on secondary
    struct ata_blkdev_state devices[4];
};



#define LEGACY_BUS_IOSTART(devnum) (((devnum)<2) ? 0x1f0 : 0x170)
#define LEGACY_ALT_IOSTART(devnum) (((devnum)<2) ? 0x3f6 : 0x376)
#define LEGACY_IRQ(devnum)         (((devnum)<2) ? 14 : 15)

#define DATA(devnum) (LEGACY_BUS_IOSTART(devnum)+0)
#define FEATERR(devnum) (LEGACY_BUS_IOSTART(devnum)+1)
#define SECTCOUNT(devnum) (LEGACY_BUS_IOSTART(devnum)+2)
#define LBALO(devnum) (LEGACY_BUS_IOSTART(devnum)+3)
#define LBAMID(devnum) (LEGACY_BUS_IOSTART(devnum)+4)
#define LBAHI(devnum) (LEGACY_BUS_IOSTART(devnum)+5)
#define DRIVEHEAD(devnum) (LEGACY_BUS_IOSTART(devnum)+6)
#define CMDSTATUS(devnum) (LEGACY_BUS_IOSTART(devnum)+7)
#define ALTCMDSTATUS(devnum) (LEGACY_ALT_IOSTART(devnum))

typedef union ata_status_reg {
    uint8_t val;
    struct {
        uint8_t err:1;  // error occurred
        uint8_t rsv1:2;
        uint8_t drq:1;  // drive has data ready or can accept data (PIO)
        uint8_t srv:1;  // overlapped mode?
        uint8_t df:1;   // drive fault
        uint8_t rdy:1;  // drive ready
        uint8_t bsy:1;  // drive busy
    };
} __packed ata_status_reg_t;

typedef union ata_cmd_reg {
    uint8_t val;
    struct {
        uint8_t rsvd1:1;
        uint8_t ien:1;   // interrupt enable - ACTIVE LOW
        uint8_t srst:1;  // software reset of all drives on bus
        uint8_t rsv2:4;
        uint8_t hob:1;   // read back high-order byte of LBA48 (?)
    } ;
} __packed ata_cmd_reg_t;

// We are still using legacy controller
static struct ata_controller_state controller;

typedef enum {OK=0, ERR, DF} ata_error_t;

/////////////////////
//
//
////////////////////


int nk_ata_pci_init(struct naut_info * naut)
{
    printk("You should print sth\n");
    INFO("init ata_pci\n");
    return 1; 
}


