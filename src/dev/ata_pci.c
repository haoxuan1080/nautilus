

#include <nautilus/nautilus.h>
#include <nautilus/blkdev.h>
#include <dev/ata_pci.h>
#include <nautilus/list.h>
#include <dev/pci.h>
#include <nautilus/mm.h>
#include <nautilus/naut_string.h>
#include <nautilus/cpu.h> //We don't know whether we need it?
#include <nautilus/irq.h> 
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

union prdt {
	uint64_t val;
	struct {
	uint32_t buffer_phys;
	uint16_t transfer_size;
	uint16_t mark_end;
	} __attribute__((packed));
}__attribute__((packed));

typedef union prdt* prdt_t;

struct ata_fn_map {
	void (*callback) (nk_block_dev_status_t, void *);
	uint64_t *context;
	uint64_t *buf;
};

struct ata_map_ring {
	struct ata_fn_map *map_ring;
	uint64_t head_pos;
	uint64_t tail_pos;
	uint64_t ring_len;
};


struct ata_blkdev_state {
    struct nk_block_dev *blkdev;

    struct pci_dev* pdev;// this is the pointer pointing to the pci device
    struct pci_bus* bus;

    spinlock_t lock;

    uint16_t data;
    uint16_t error;
    uint16_t sector_count;
 
 	union {
		uint16_t sector_num;
		uint16_t lba_lo ;
	};
	union {
		uint16_t cylinder_low;
		uint16_t lba_mid ;
	};
	union {
		uint16_t cylinder_high;
		uint16_t lba_high;
	};
	union {
		uint16_t drive;
		uint16_t head;
	};
	union {
		uint16_t command;
		uint16_t status;
	};
	union {
		uint16_t control;
		uint16_t alt_status;
	};
    
    uint32_t bar4; //bar4 for legacy mode
    uint32_t BMR_cmd;
    uint32_t BMR_prdt;
    uint32_t BMR_status;
    
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
    
    //add interrupt variables
    uint8_t pci_intr;
    uint8_t intr_vec;
    uint8_t intr_flag;
    uint8_t* source;
    uint8_t* dest;

    struct ata_map_ring *tx_map;
    struct ata_map_ring *rx_map;

};

uint8_t* source_global;
uint8_t* dest_global;

struct ata_controller_state {
    // devices 0,1 are master/slave on primary
    // devices 2,3 are master/slave on secondary
    struct ata_blkdev_state devices[4];
};

//Map ring length declaration constant
#define TX_DSC_COUNT 128
#define RX_DSC_COUNT 128

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
        //DEBUG("In ata wait, the value of the status register is: %x\n", stat.val);
        if (stat.err) {
	    int err = inb(s->error);
            ERROR("Controller error (0x%x)\n",stat.val);
	    ERROR("The ERR Register is: %x\n", err);
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
    outb(0, s->sector_count);
    outb(0, s->lba_lo);
    outb(0, s->lba_mid);
    outb(0, s->lba_high);
    outb(0xec, s->command); // IDENTIFY

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
        buf[j] = inw(s->data);
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

static int ata_lba48_read_write_dma(void* state,
		                    uint64_t block_num,
				    uint8_t *buf,
				    int write)
{
    

    struct ata_blkdev_state *s = (struct ata_blkdev_state *) state;

    DEBUG("s->BMR_cmd = %x\n", s->BMR_cmd);
    DEBUG("s->BMR_status = %x\n", s->BMR_status);
    DEBUG("s->BMR_prdt = %x\n", s->BMR_prdt);  //make sure nothing crashes the memory
    DEBUG("prdt_phys = %p\n", s->prdt_phys);


    uint8_t sectcnt[2];
    uint8_t lba[7]; // we use 1..6 as per convention...

    lba[6] = (block_num >> 40) & 0xff;
    lba[5] = (block_num >> 32) & 0xff;
    lba[4] = (block_num >> 24) & 0xff;
    lba[3] = (block_num >> 16) & 0xff;
    lba[2] = (block_num >>  8) & 0xff;
    lba[1] = (block_num >>  0) & 0xff;

    if (ata_wait(s,0)) {
        ERROR("Wait failed - resetting drive\n");
        ata_reset(s);
        return -1;
    }

    DEBUG("Write a 0x04 to BMR Status\n");
    outb(0x4, s->BMR_status);
    
    inb(s->alt_status); // do this multiple times to consume 400ns
    inb(s->alt_status); // do this multiple times to consume 400ns
    inb(s->alt_status); // do this multiple times to consume 400ns
    inb(s->alt_status); // do this multiple times to consume 400ns

    int BMR_status = inb(s->BMR_status);
    DEBUG("The status after writing 0x4 is: %x\n", BMR_status);

    outb(0, s->BMR_cmd);
    outl((uint32_t)(uint64_t)s->prdt_phys, s->BMR_prdt);
    outb(0xe0 | (s->id << 4) | lba[6] & 0xf0 >> 4, s->head);//probably this selected the dirve, 
    DEBUG("The head register is %x\n", inb(s->head));
    outb(0,s->sector_count);
    outb(lba[4],s->lba_lo);
    outb(lba[5],s->lba_mid);
    outb(lba[6],s->lba_high);
    outb(1,s->sector_count);
    outb(lba[1],s->lba_lo); //How does it write 16-bit data to the register with outb?
    outb(lba[2],s->lba_mid);
    outb(lba[3],s->lba_high);
    
    DEBUG("LBA and sector count completed\n");

    int status = inb(s->BMR_status); 

    if (write) {
        memcpy(s->mem_buffer, buf, 512);
        outb(0x35, s->command);// maybe DMA first
        outb(0x1, s->BMR_cmd);
    }
    else {
        outb(0x25, s->command);
	outb(0x8 | 0x1, s->BMR_cmd);
    }

    DEBUG("drive status is %x\n", inb(s->status));
    DEBUG("BMR status is %x\n", inb(s->BMR_status));
    return 0;
}

static int ata_unmap_callback(struct ata_map_ring* map,
                                uint64_t** callback,
                                void** context, void** buf)
{
  // callback is a function pointer
  DEBUG("unmap callback fn head_pos %d tail_pos %d\n", map->head_pos, map->tail_pos);
  if(map->head_pos == map->tail_pos) {
    // if there is an empty mapping ring buffer, do not unmap the callback
    ERROR("Try to unmap an empty queue\n");
    return -1;
  }

// WHY DO WE NEED TO USE HEAD POS*******
  uint64_t i = map->head_pos;
  // TODO(panitan)
  // uint64_t i = map->tail_pos-1;
  DEBUG("the map_callback in unmap is 0x%p\n",map->map_ring[i].callback);
  *callback = (uint64_t *) map->map_ring[i].callback;
  *context =  map->map_ring[i].context;
  *buf = map->map_ring[i].buf;
  map->map_ring[i].callback = NULL;
  map->map_ring[i].context = NULL;
  map->map_ring[i].buf = NULL;
  map->head_pos = (1 + map->head_pos) % map->ring_len;
  DEBUG("end unmap callback fn head_pos %d tail_pos %d\n", map->head_pos, map->tail_pos);
  return 0;
}

static int ata_map_callback(struct ata_map_ring* map,
                              void (*callback)(nk_block_dev_status_t, void*),
                              void* context,void* buf)
{
  DEBUG("map callback head_pos %d tail_pos %d\n", map->head_pos, map->tail_pos);
  if(map->head_pos == ((map->tail_pos + 1) % map->ring_len)) {
    // when the mapping callback queue is full
    ERROR("Callback mapping queue is full.\n");
    return -1;
  }
  DEBUG("tail_pos %d\n", map->tail_pos);
  uint64_t i = map->tail_pos;
  struct ata_fn_map* fnmap = (map->map_ring + i);
   fnmap->callback = callback;
   fnmap->context = (uint64_t *)context;
   fnmap->buf = (uint64_t *) buf;
  map->tail_pos = (1 + map->tail_pos) % map->ring_len;
  DEBUG("mapped callback head_pos: %d, tail_pos: %d\n", map->head_pos, map->tail_pos);
  return 0;
}


static int read_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *dest,void (*callback)(nk_block_dev_status_t, void *), void *context)
{
/*    STATE_LOCK_CONF;
    struct ata_blkdev_state *s = (struct ata_blkdev_state *)state;
    DEBUG("read_blocks on device %s starting at %lu for %lu blocks\n",
          s->blkdev->dev.name, blocknum, count);
    STATE_LOCK(s);
    if (blocknum+count >= s->num_blocks) {
        STATE_UNLOCK(s);
        ERROR("Illegal access past end of disk\n");
        return -1;
    } else {
        int rc =0;
	DEBUG("The count value for dest is %d\n",count);
	for (int i = 0; i<count; i++) {
	    DEBUG("read on device %s block num %u\n", s->blkdev->dev.name, i);

            rc += ata_lba48_read_write_dma(s,blocknum+i*512, dest+i*512, 0);
	}
	
        STATE_UNLOCK(s);
        if (callback) {
            callback(NK_BLOCK_DEV_STATUS_SUCCESS,context);
        }
        return rc;
    } */
struct ata_blkdev_state *s = (struct ata_blkdev_state *)state;
    DEBUG("the head_pos_tx_1 is %x\n",s->tx_map->head_pos);
    DEBUG("the head_pos_rx_1 is %x\n",s->rx_map->head_pos);

    DEBUG("\n\n\n--------------------------------------------------------\n");
    DEBUG("read_blocks on device %s starting at %lu for %lu blocks\n",
          s->blkdev->dev.name, blocknum, count);
    //STATE_LOCK(s);
    if (blocknum+count >= s->num_blocks) {
    //    STATE_UNLOCK(s);
        ERROR("Illegal access past end of disk\n");
        return -1;
    }
    int rc =0;
    //Always map callback
    DEBUG("read the ata call back 0x%p\n",callback);

    rc = ata_map_callback(((struct ata_blkdev_state*)state)->rx_map,callback,context,dest);
    //STATE_LOCK(s);
    if (!rc) {
    rc += ata_lba48_read_write_dma(s,blocknum,dest,0);
    }
    DEBUG("mapped callback in write_blocks head_pos: %d, tail_pos: %d\n", s->tx_map->head_pos, s->tx_map->tail_pos);

    return rc;
}

static int write_blocks(void *state, uint64_t blocknum, uint64_t count, uint8_t *src,void (*callback)(nk_block_dev_status_t, void *), void *context)
{
   // DEBUG("the head_pos_tx_1 is %x\n",s->tx_map->head_pos);
   // DEBUG("the head_pos_rx_1 is %x\n",s->rx_map->head_pos);

  //  STATE_LOCK_CONF;
    struct ata_blkdev_state *s = (struct ata_blkdev_state *)state;
    DEBUG("the head_pos_tx_1 is %x\n",s->tx_map->head_pos);
    DEBUG("the head_pos_rx_1 is %x\n",s->rx_map->head_pos);

    DEBUG("\n\n\n--------------------------------------------------------\n");
    DEBUG("write_blocks on device %s starting at %lu for %lu blocks\n",
          s->blkdev->dev.name, blocknum, count);
    //STATE_LOCK(s);
    if (blocknum+count >= s->num_blocks) {
    //    STATE_UNLOCK(s);
        ERROR("Illegal access past end of disk\n");
        return -1;
    }
    int rc =0;
    //Always map callback
    DEBUG("read the ata call back 0x%p\n",callback);

    rc = ata_map_callback(((struct ata_blkdev_state*)state)->tx_map,callback,context, src);
    //STATE_LOCK(s);
    if (!rc) {
    rc += ata_lba48_read_write_dma(s,blocknum,src,1);
    }
    DEBUG("End of the read function\n");

  /*
    else {
        int rc =0;
	DEBUG("the count value is %d\n",count);
        for (int i = 0; i<count; i++) {
            source_global = src; 
	    DEBUG("the source value is %x\n",source_global);
            DEBUG("write on device %s block num %u\n", s->blkdev->dev.name, i);

            rc += ata_lba48_read_write_dma(s,blocknum+i*512, src+i*512, 1);
        } 
	
        STATE_UNLOCK(s);
        if (callback) {
            callback(NK_BLOCK_DEV_STATUS_SUCCESS,context);
        }
        return rc;	
    }
  */
    DEBUG("mapped callback in write_blocks head_pos: %d, tail_pos: %d\n", s->tx_map->head_pos, s->tx_map->tail_pos);
	return rc;
}

static int get_characteristics(void *state, struct nk_block_dev_characteristics *c)
{
    STATE_LOCK_CONF;
    struct ata_blkdev_state *s = (struct ata_blkdev_state *)state;

    STATE_LOCK(s);
    c->block_size = s->block_size;
    c->num_blocks = s->num_blocks;
    STATE_UNLOCK(s);
    return 0;

}

static struct nk_block_dev_int inter =
{
    .get_characteristics = get_characteristics,
    .read_blocks = read_blocks,
    .write_blocks = write_blocks,
};

//union prdt prdt_glb[65536];

static void ata_device_addr_init(int devnum, void* dev)
{
    struct ata_blkdev_state *s = (struct ata_blkdev_state *) dev;
    s->data = DATA(devnum);
    s->error = s->data+1;
    s->sector_count = s->data + 2;
    s->lba_lo = s->data + 3;
    s->lba_mid = s->data + 4;
    s->lba_high = s->data + 5;
    s->head = s->data + 6;
    s->command = s->data + 7;
    s->control = ALTCMDSTATUS(devnum);



    //We need to find bar4 though pci and then assign address of BMR
    //

    //We seems to assign the correct bar4 to it, and the pdev seems correct
    s->bar4 = s->pdev->cfg.dev_cfg.bars[4] & 0xfffffffc;// this is IO port
    DEBUG("BAR4 from structure is: %x\n", s->bar4);
    //s->bar4 = pci_cfg_readl(s->bus->num, s->pdev->num, 0, 0x20);
    //DEBUG("BAR4 from pci_read is: %x\n", s->bar4);
    s->BMR_cmd = s->bar4 + s->channel*8;
    s->BMR_status = s->BMR_cmd + 2; 
    s->BMR_prdt = s->BMR_cmd + 4;

    s->prdt = (void*)malloc(4*sizeof(*(s->prdt)));
    memset(s->prdt, 0, 4*sizeof(*(s->prdt)));
    //s->prdt = &prdt_glb[0];
    s->prdt_phys = (uint8_t*)s->prdt;// this actually virtual address 
    s->mem_buffer = (void*)malloc(4096);
    memset(s->mem_buffer, 0, 4096);
    s->prdt[0].buffer_phys = (uint32_t)s->mem_buffer;//we cast pointer to 32 bit
    DEBUG("The lowest byte in prdt[0] is %x\n", *(char*)s->prdt[0].buffer_phys);
    DEBUG("");
    DEBUG("The mem_buffer address is %x\n", s->prdt[0].buffer_phys);
    DEBUG("PRDT base address: %x assigned to drive %u\n", s->prdt_phys, devnum);
    s->prdt[0].transfer_size = 512; //We just assume sector size is 512 bytes
    s->prdt[0].mark_end = 0x8000; //The MSB is set 1 so this marks the end of PRDT
    //s->prdt[0].val |= 0x8000000000000000UL;
    DEBUG("interrupt pin is %x\n",s->pdev->cfg.dev_cfg.intr_pin);
    DEBUG("interrupt line is %x\n",s->pdev->cfg.dev_cfg.intr_line);
    s->pci_intr = s->pdev->cfg.dev_cfg.intr_pin;
//	    DEBUG("the BMR_status is %x\n",inb(dev->BMR_status));
    if (devnum < 2) {
	    s->intr_vec = 14;
    }
    else 
	    s->intr_vec = 15;
    s->intr_flag =0;

    s->tx_map = malloc(sizeof(*(s->tx_map)));
    s->tx_map->map_ring = malloc(TX_DSC_COUNT*sizeof(*(s->tx_map->map_ring)));
    s->rx_map = malloc(sizeof(*(s->rx_map)));
    s->rx_map->map_ring = malloc(RX_DSC_COUNT*sizeof(*(s->rx_map->map_ring)));
    s->rx_map->ring_len = RX_DSC_COUNT;
    s->tx_map->ring_len = TX_DSC_COUNT;
    s->tx_map->tail_pos = 0;
    s->rx_map->tail_pos = 0;
    s->tx_map->head_pos = 0;
    s->rx_map->head_pos = 0;
}

static int ata_pci_irq_handler(excp_entry_t* excp, excp_vec_t vec, void *s)
{
	//Empty irq handler function!
	struct ata_blkdev_state *dev = (struct ata_blkdev_state *) s;
        uint8_t flag = 0;	
	DEBUG("the status register in IRQ is %x\n",inb(dev->BMR_status));
	DEBUG("IRQ Handler here!\n");
	if (inb(dev->BMR_status) & 0x04)
	      	outb(inb(dev->BMR_status) | 0x04, dev->BMR_status);
	DEBUG("the devnum is %d\n",dev->channel*2 + dev->id);
	DEBUG("the status register After clearing IRQ bit is %x\n",inb(dev->BMR_status));
	if (inb(dev->BMR_cmd) != 0) {
	nk_block_dev_status_t status = NK_BLOCK_DEV_STATUS_SUCCESS;
	void * buf = NULL;
	void (*callback) (nk_block_dev_status_t, void*) = NULL;
        void *context = NULL;

	if (inb(dev->BMR_cmd) == 1 & (!(inb(dev->BMR_status) & 0x01))) {
	       	// Check BMR_status too??
		flag = 0;
		outb(0,dev->BMR_cmd);
		DEBUG("actual IRQ Handler for write\n");
		DEBUG("unmapping the write callback\n");
		ata_unmap_callback(dev->tx_map,(uint64_t **) &callback, (void **)&context, (void **) &buf);
		}
	else if  (inb(dev->BMR_cmd) == 9 & (!(inb(dev->BMR_status) & 0x01))) {
		flag = 1;
		outb(0,dev->BMR_cmd);
                DEBUG("actual IRQ Handler for read\n");
                nk_block_dev_status_t status = NK_BLOCK_DEV_STATUS_SUCCESS;
                DEBUG("unmapping the write callback\n");
                ata_unmap_callback(dev->rx_map,(uint64_t **) &callback, (void **)&context, (void **) &buf);

	}
	while (1) {
                  int d_status = inb(dev->status);
                  if (!((d_status & 0x80) || (d_status & 0x08)))
                          break;
                  }
        int temp_status = inb(dev->BMR_status);
        if (temp_status & 0x02) {
        	status = NK_BLOCK_DEV_STATUS_ERROR;
        }
	if (flag == 1) {
		memcpy(buf, dev->mem_buffer, 512);
		//uint8_t* temp_buf = (uint8_t*) buf;
		/*for (int i = 0; i < 512; i++) {
            		printk("%c",temp_buf[i]);
            		if (i%64 == 0) {
                		printk("\n");
                	}
           	}*/

	}	
	DEBUG("Interrupt serviced, calling back!\n");
        DEBUG("the callback value in IRQ is 0x%p\n",callback);


        if (callback) {
        	DEBUG("invoking callback function: 0x%p\n", callback);
                callback(status,context);
        }
}
	IRQ_HANDLER_END();
	return 0;
}

static void discover_device(int channel, int id, struct pci_bus *bus, struct pci_dev *pdev)
{
    int devnum = channel*2 + id;
    struct ata_blkdev_state *s = &(controller.devices[devnum]);

    s->bus = bus;
    s->pdev = pdev;

    DEBUG("Considering device ata0-%d-%d\n",channel,id);

    spinlock_init(&s->lock); //do we really need this while booting
    ata_device_addr_init(devnum, s);
    DEBUG("the head_pos_tx_1 is %x\n",s->tx_map->head_pos);
    DEBUG("the head_pos_rx_1 is %x\n",s->rx_map->head_pos);


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
            DEBUG("Failed totx_ identify device\n");
        }
    } else {
        DEBUG("Nonexistent or unsupported device type detected\n");
    }
    //register the irq handler
    if (devnum % 2 == 0) {
    	register_irq_handler(s->intr_vec,ata_pci_irq_handler,s);
    	nk_unmask_irq(s->intr_vec);
    }
}



static int discover_ata_drives(struct naut_info * naut)
{
    memset((void*)&controller,0,sizeof(controller));

    struct pci_info *pci = naut->sys.pci;
    struct list_head *curbus, *curdev;
    struct pci_bus *bus = NULL;
    struct pci_dev *pdev = NULL;
    uint32_t num = 0;

    INFO("init\n");

    struct list_head dev_list;

    INIT_LIST_HEAD(&dev_list);

    if (!pci) {
        ERROR("No PCI info\n");
        return -1;
    }

    list_for_each(curbus,&(pci->bus_list)) {
        bus = list_entry(curbus,struct pci_bus,bus_node);

        DEBUG("Searching PCI bus %u for IDE devices\n", bus->num);

        list_for_each(curdev, &(bus->dev_list)) {
            pdev = list_entry(curdev,struct pci_dev,dev_node);
	    DEBUG("Bus num is: %x\n", bus->num);
	    DEBUG("pdev num is: %x\n", pdev->num);
            struct pci_cfg_space *cfg = &pdev->cfg;
	    DEBUG("Device %u is a 0x%x:0x%x\n", pdev->num, cfg->vendor_id, cfg->device_id);
	    if (cfg->class_code == 0x01 && cfg->subclass == 0x01){
		    DEBUG("IDE Device Found\n");
		    break;
	    }
            
        }
            if (pdev->cfg.class_code == 0x01 && pdev->cfg.subclass == 0x01){
		    DEBUG("IDE Device Found\n");
		    break;
	    }

    }

    uint32_t pci_command_reg = pci_cfg_readl(bus->num, pdev->num, pdev->fun, 0x04);
    if(!(pci_command_reg & (1 << 2))) {
        pci_command_reg |= (1 << 2);
    }
    pci_cfg_writel(bus->num, pdev->num, pdev->fun, 0x04, pci_command_reg);


    //Problem: This is the previous device, Why?

    DEBUG("BAR4 from structure is: %x\n", pdev->cfg.dev_cfg.bars[4]);
    uint32_t bar4 = pci_cfg_readl(bus->num, pdev->num, pdev->fun, 0x20);
    DEBUG("BAR4 from pci_read is: %x\n", bar4);
    uint32_t class = pci_cfg_readl(bus->num, pdev->num, pdev->fun, 0x08);
    DEBUG("Class and sub class double word is: 0x%x\n", class);


    //PCI staff ends, discover device 
    discover_device(0,0, bus, pdev);
    discover_device(0,1, bus, pdev);
    discover_device(1,0, bus, pdev);
    discover_device(1,1, bus, pdev);

    return 0;
}

//src/arch/x64/init.c
int nk_ata_pci_init(struct naut_info * naut)
{
    printk("You should print sth\n");
    INFO("init ata_pci\n");
    return discover_ata_drives(naut); 
}


