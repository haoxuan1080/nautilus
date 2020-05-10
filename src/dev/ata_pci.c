

#include <nautilus/nautilus.h>
#include <nautilus/blkdev.h>
#include <dev/ata_pci.h>

//#ifndef NAUT_CONFIG_DEBUG_ATA_PCI
//#undef DEBUG_PRINT
//#define DEBUG_PRINT(fmt, args...)
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
/////////////////////

static ata_error_t ata_wait(struct ata_blkdev_state *s, int data)
{
    uint8_t devnum = s->channel * 2 + s->id;
    ata_status_reg_t stat;

    DEBUG("Waiting on drive %u for %s\n",devnum,data ? "data" : "command");

    while (1) {
        stat.val = inb(CMDSTATUS(devnum));
        if (stat.err) {
            ERROR("Controller error (0x%x)\n",stat.val);
            return ERR;
        }
        if (stat.df) {
            ERROR("Drive fault (0x%x)\n",stat.val);
            return DF;
        }
        if (!stat.bsy && (!data || stat.drq)) {
            DEBUG("Leaving wait with status=0x%x\n",stat.val);
            return OK;
        }
    }
}

static int ata_reset(struct ata_blkdev_state *s)
{
    uint8_t devnum = s->channel * 2 + s->id;
    ata_cmd_reg_t c;

    DEBUG("reset of drive %u\n", devnum);

    c.val=0;
    c.ien=1; // disable interrupts
    c.srst=1; // start resetting

    outb(c.val,CMDSTATUS(devnum));
    //We placed 4 read insturctions here, delay but didn't changed the detected device number 
    c.srst=0; // stop resetting

    outb(c.val,CMDSTATUS(devnum));

    return 0;
}


static int ata_drive_select(struct ata_blkdev_state *s)
{
    uint8_t devnum = s->channel * 2 + s->id;

    DEBUG("Drive select %u\n",devnum);

    outb(0xa0 | s->id << 4, DRIVEHEAD(devnum));
    inb(ALTCMDSTATUS(devnum)); // do this multiple times to consume 400ns
    inb(ALTCMDSTATUS(devnum)); // do we have to do this in initialization?
    inb(ALTCMDSTATUS(devnum));
    inb(ALTCMDSTATUS(devnum));
    inb(ALTCMDSTATUS(devnum)); // do this multiple times to consume 400ns
    inb(ALTCMDSTATUS(devnum)); // do we have to do this in initialization?
    inb(ALTCMDSTATUS(devnum));
    inb(ALTCMDSTATUS(devnum));

    return 0;
}

static int ata_drive_detect(struct ata_blkdev_state *s)
{
    uint8_t devnum = s->channel * 2 + s->id;
    uint16_t t;

    DEBUG("drive detect for device %u\n",devnum);

    ata_reset(s);
    ata_drive_select(s);
    t  = inb(LBAMID(devnum));
    t |= ((uint16_t)inb(LBAHI(devnum)))<<8;

    DEBUG("drive reports type 0x%x\n",t);

    if (t==0xeb14 || t==0x9669) {
        s->type=CD;
    }
    if (t==0x0 || t==0xc33c) {
        s->type=HD;
    }

    DEBUG("devnum %u detected as type 0x%x (%s-%s)\n",
         devnum, t, s->type==CD ? "CD" : s->type==HD ? "HD" : "NONE",
         t==0xeb14 ? "PATAPI" : t==0x9669 ? "SATAPI" :
         t==0x0 ? "PATA" : t==0xc33c ? "SATA" : "UNKNOWN");

    return 0;
}

static int ata_drive_identify(struct ata_blkdev_state *s)
{
    uint8_t devnum = s->channel * 2 + s->id;
    uint16_t t;

    DEBUG("Identify drive %u\n",devnum);

    ata_reset(s);
    ata_drive_select(s);
    outb(0,SECTCOUNT(devnum));
    outb(0,LBALO(devnum));
    outb(0,LBAMID(devnum));
    outb(0,LBAHI(devnum));
    outb(0xec, CMDSTATUS(devnum)); // IDENTIFY

    if (!inb(CMDSTATUS(devnum))) {
        // nonexistent drive... why am I identifying it?
	// Why could we detect something but it is identified as noexistent?
        DEBUG("Drive is nonexistent\n");
        return -1;
    }

    if (ata_wait(s,0)) {
        ERROR("Poll in identify failed... (BSY)\n");
        return -1;
    }

    if (inb(LBAMID(devnum)) || inb(LBAHI(devnum))) {
        ERROR("Not an ATA drive or a flakey ATAPI drive\n");
        return -1;
    }

    if (ata_wait(s,1)) {
        ERROR("Poll in identify failed... (DRQ)\n");
        return -1;
    }

    

    uint16_t buf[256];
    int j;

    DEBUG("Acquiring identity block from drive\n");

    for (j=0;j<256;j++) {
        buf[j] = inw(DATA(devnum));
    }
    if (!((buf[83] >> 10) & 0x1)) {
        ERROR("LBA48 not supported on this drive\n");
        return -1;
    } else {
        s->block_size = 512;
        s->num_blocks =
            (((uint64_t) buf[103]) << 48) +
            (((uint64_t) buf[102]) << 32) +
            (((uint64_t) buf[101]) << 16) +
            (((uint64_t) buf[100]) <<  0) ;
        DEBUG("LBA48 supported, block data is %x %x %x %x\n",
              buf[103], buf[102], buf[101], buf[100]);
        DEBUG("Interpretted as numblocks=0x%x\n",s->num_blocks);
        return 0;
    }
}

static int read_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *dest,void (*callback)(nk_block_dev_status_t, void *), void *context)
{
   return 0;
}

static int write_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *src,void (*callback)(nk_block_dev_status_t, void *), void *context)
{
    return 0;
}

static int get_characteristics(void *state, struct nk_block_dev_characteristics *c)
{
    return 0;
}

static struct nk_block_dev_int inter =
{
    .get_characteristics = get_characteristics,
    .read_blocks = read_blocks,
    .write_blocks = write_blocks,
};


static void discover_device(int channel, int id)
{
    int devnum = channel*2 + id;
    struct ata_blkdev_state *s = &(controller.devices[devnum]);

    DEBUG("Considering device ata0-%d-%d\n",channel,id);

    spinlock_init(&s->lock); //do we really need this while booting

    s->channel = channel;
    s->id = id;
    s->controller = &controller;

    ata_drive_detect(s);

    INFO("finished drive_detect for device ata0-%d-%d\n",channel,id);

    if (s->type!=NONE) {
        if (!ata_drive_identify(s)) {
            char name[32];
            sprintf(name,"ata0-%d-%d",channel,id);
            s->blkdev = nk_block_dev_register(name, 0, &inter, s);
            if (!s->blkdev) {
                ERROR("Failed to register %s\n",name);
            }
            INFO("Added ata device %s, type %s, blocksize=%lu, numblocks=%lu\n",
                 name,
                 s->type==HD ? "HD" : s->type==CD ? "CD" : "UNKNOWN",
                 s->block_size,s->num_blocks );
        } else {
            DEBUG("Failed to identify device\n");
        }
    } else {
        DEBUG("Nonexistent or unsupported device type detected\n");
    }

}

static int discover_ata_drives()
{
    memset((void*)&controller,0,sizeof(controller));

    discover_device(0,0);
    discover_device(0,1);
    discover_device(1,0);
    discover_device(1,1);

    return 0;
}

//src/arch/x64/init.c
int nk_ata_pci_init(struct naut_info * naut)
{
    printk("You should print sth\n");
    INFO("init ata_pci\n");
    return discover_ata_drives(); 
}


