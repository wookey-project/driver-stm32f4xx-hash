#include "api/libhash.h"
#include "hash_regs.h"
#include "libc/syscall.h"
#include "libc/stdio.h"
#include "libc/nostd.h"
#include "libc/string.h"
#include "libc/regutils.h"
#include "libc/arpa/inet.h"

#define CONFIG_USR_DRV_HASH_DEBUG 0

#define DMA_MAX_TRANSFERT_SIZE 0x8000

#define DMA_HASH_CTRL CRYP_USER_DMA_CTRL
#define DMA_STREAM_HASH_IN HASH_DMA_IN_STREAM
#define DMA_CHANNEL_HASH_IN HASH_DMA_IN_CHANNEL


static volatile bool use_dma = false;
static volatile bool use_it  = false;

static volatile int dma_hash_desc = 0;
static volatile int dev_hash_desc = 0;
static volatile dma_t dma_hash = { 0 };
static volatile device_t dev_hash;

static volatile bool hash_is_mapped = false;


static volatile cb_endofdigest eodigest_cb = NULL;
static volatile cb_endofdma  eodma_cb = NULL;
static volatile bool dma_end = false;

void hash_wait_dma_end(void)
{
    while (!dma_end) {}
    dma_end = 0;
}

int hash_map(void)
{
    if (hash_is_mapped == false) {
#if CONFIG_USR_DRV_HASH_DEBUG
        printf("Mapping hash\n");
#endif
        uint8_t ret;
        ret = sys_cfg(CFG_DEV_MAP, dev_hash_desc);
        hash_is_mapped = true;
        if (ret != SYS_E_DONE) {
#if CONFIG_USR_DRV_HASH_DEBUG
            printf("Unable to map hash!\n");
#endif
            goto err;
        }
    }

    return 0;
err:
    return -1;
}

int hash_unmap(void)
{
    if (hash_is_mapped) {
#if CONFIG_USR_DRV_HASH_DEBUG
        printf("Unmapping hash\n");
#endif
        uint8_t ret;
        ret = sys_cfg(CFG_DEV_UNMAP, dev_hash_desc);
        hash_is_mapped = false;
        if (ret != SYS_E_DONE) {
#if CONFIG_USR_DRV_HASH_DEBUG
            printf("Unable to unmap hash!\n");
#endif
            goto err;
        }
    }

    return 0;
err:
    return -1;
}

static int hash_send_buf_dma(physaddr_t buff, uint32_t size)
{
    uint8_t ret;

#if CONFIG_USR_DRV_HASH_DEBUG
    printf("sending %x bytes from @%x through DMA\n", size, buff);
#endif
    dma_hash.in_addr = buff;
    dma_hash.size    = size;

    ret = sys_cfg(CFG_DMA_RECONF, (dma_t*)&dma_hash,
                  (DMA_RECONF_BUFIN | DMA_RECONF_BUFOUT | DMA_RECONF_BUFSIZE),
                   dma_hash_desc);

    if (ret != SYS_E_DONE) {
#if CONFIG_USR_DRV_HASH_DEBUG
        printf("Unable to launch DMA!\n");
#endif
        goto err;
    }
    return 0;

err:
    return -1;
}

/* [RB] FIXME: the 'no dma' version of the hash coprocessor is a work in
 * progress and has not been extensively tested (compared to the DMA version).
 */
static void hash_send_buf_nodma(physaddr_t buff, uint32_t size)
{
    uint32_t *tab = (uint32_t*)buff;
    uint32_t offset;
    /* buff is naturally truncated to a multiple of 4 bytes here */
    for (offset = 0; offset < (size / 4); ++offset) {
        write_reg_value(_r_CORTEX_M_HASH_DIN, tab[offset]);
    }
    /* residual? size not a multiple of 4 ? */
    uint8_t *last = (uint8_t*)(buff+(4*offset));
    uint32_t tmp;
    switch(size & 3){
      case 1:
         write_reg_value(_r_CORTEX_M_HASH_DIN, *((uint8_t *)last)); 
         break;
      case 2:
         write_reg_value(_r_CORTEX_M_HASH_DIN, *((uint16_t *)last)); 
         break;
      case 3:
         tmp  = ((uint8_t*) last)[0];
         tmp |= ((uint8_t*) last)[1] << 8;
         tmp |= ((uint8_t*) last)[2] << 16;
         write_reg_value(_r_CORTEX_M_HASH_DIN, tmp);  
         break;
      default:
         break;
    }
}


static void dma_hash_handler(uint8_t irq,
                             uint32_t status)
{
    if (eodma_cb) {
        eodma_cb(irq, status);
    }
    dma_end = true;
}

static void hash_handler(uint8_t irq,
                         uint32_t status,
                         uint32_t data __attribute__((unused)))
{
    /* executing the user callback at ISR time when DCIE interrupt rise */
    if (eodigest_cb) {
        eodigest_cb(irq, status);
    }
}

int hash_finalize(void)
{
    return 0;
}

int hash_request(hash_req_type_t      type,
                     uint32_t             addr,
                     uint32_t             size)
{
    /* NOTE: the hash hardware IP on STM32F4xx is meant to
     * process intermediate chunks of multiple of 32 bits (HASH FIFO size words).
     */
    if((type != HASH_REQ_LAST) && ((size % 4) != 0)){
#if CONFIG_USR_DRV_HASH_DEBUG
        printf("HASH Error: asking for intermediate size of %d not 32-bit word multiple!\n", size);
#endif
        goto err;
    }
    /* If size is not 512 multiple, setting DCAL to 1 automatically
     * pads to finish the last chunk calculation if needed and the
     * result is calculated */
    if(!use_dma){
        hash_send_buf_nodma(addr, size);
        if(type == HASH_REQ_LAST){
  	    /* Handle NBLW for the last word (to handle padding issues) */
            set_reg(_r_CORTEX_M_HASH_STR, (8*(size % 4)), HASH_STR_NBLW);
            /* last request, set DCAL to 1 to perform digest closure */
            set_reg(_r_CORTEX_M_HASH_STR, 0x1, HASH_STR_DCAL);
        }
    }
    else{
        uint32_t residue = size;
        uint32_t offset = 0;

	/* Sanity checks */
	if((addr % 4) != 0){
	    /* DMA buffer must be aligned on 4 bytes */
#if CONFIG_USR_DRV_HASH_DEBUG
            printf("HASH Error: asking for DMA on unaligned address %xn", addr);
#endif
	    goto err;
        }
	if((DMA_MAX_TRANSFERT_SIZE % 4) != 0){
		goto err;
	}
        if (size > DMA_MAX_TRANSFERT_SIZE) {
            do {
		if((type != HASH_REQ_LAST) || (residue != 0)){
	                set_reg(_r_CORTEX_M_HASH_CR, 0x1, HASH_CR_MDMAT);
		}
		else{
			if(residue == 0){
				/* Handle NBLW for the last word (to handle padding issues) */
        			set_reg(_r_CORTEX_M_HASH_STR, (8*(size % 4)), HASH_STR_NBLW);
	        		set_reg(_r_CORTEX_M_HASH_CR, 0x0, HASH_CR_MDMAT);
			}
		}
                set_reg(_r_CORTEX_M_HASH_CR, 0x1, HASH_CR_DMAE);
                if(hash_send_buf_dma(addr+offset, DMA_MAX_TRANSFERT_SIZE)){
                    goto err;
                }
                hash_wait_dma_end();
                residue -= DMA_MAX_TRANSFERT_SIZE;
                offset += DMA_MAX_TRANSFERT_SIZE;
            } while (residue > DMA_MAX_TRANSFERT_SIZE);
        }

        /* Handle the residue if necessary */
        if(residue != 0){
    	    if(type == HASH_REQ_LAST){
		/* Handle NBLW for the last word (to handle padding issues) */
        	set_reg(_r_CORTEX_M_HASH_STR, (8*(size % 4)), HASH_STR_NBLW);
		/* Set MDMAT to 0, DCAL should be automatically set to 1 at the end the
		 * next DMA transfer
		 */
	        set_reg(_r_CORTEX_M_HASH_CR, 0x0, HASH_CR_MDMAT);
	    }
	    else{
	    	set_reg(_r_CORTEX_M_HASH_CR, 0x1, HASH_CR_MDMAT);
	    }
            set_reg(_r_CORTEX_M_HASH_CR, 0x1, HASH_CR_DMAE);
	    if((residue % 4) != 0){
		/* NOTE: this is a little bit ugly since we overflow in reading the input buffer
		 * because of the limitations of the hardware taking only 32 bits words at a time,
		 * yielding in DMA transfers multiple of 32 bits.
		 */
            	if(hash_send_buf_dma(addr+offset, residue + 4)){
                    goto err;
                }
	    }
	    else{
            	if(hash_send_buf_dma(addr+offset, residue)){
                    goto err;
                }
	    }
            hash_wait_dma_end();
        }
    }
    return 0;

err:
    return -1;
}

int hash_init(cb_endofdigest eodigest_callback,
                  cb_endofdma    eodma_callback,
                  hash_algo_t algo)
{

    if (!hash_is_mapped) {
        uint8_t ret;
        ret = sys_cfg(CFG_DEV_MAP, dev_hash_desc);
        hash_is_mapped = true;
        if (ret != SYS_E_DONE) {
#if CONFIG_USR_DRV_HASH_DEBUG
            printf("Unable to map hash device!\n");
#endif
            goto err;
        }
    }

    uint32_t reg = 0;
    /* registering the End Of Digest callback */
    if (eodigest_callback) {
        eodigest_cb = eodigest_callback;
    }
    if (eodma_callback) {
        eodma_cb = eodma_callback;
    }

    /* configure the HASH device, depending on the choosen configuration */

    /* datatype mode (bit-swapping, depending on the input data type, see
     * STM-RM0090 chap 25.3.2 */
    /* byte-based little to big translation */
    set_reg(&reg, 2, HASH_CR_DATATYPE);

    switch (algo) {
        case HASH_SHA1:
            /* algo[0:1] == 0 */
            set_reg(&reg, 0, HASH_CR_ALGO0);
            set_reg(&reg, 0, HASH_CR_ALGO1);
            set_reg(&reg, 0, HASH_CR_MODE);
            break;
        case HASH_MD5:
            set_reg(&reg, 1, HASH_CR_ALGO0);
            set_reg(&reg, 0, HASH_CR_ALGO1);
            set_reg(&reg, 0, HASH_CR_MODE);
            break;
        case HASH_SHA224:
	    set_reg(&reg, 0, HASH_CR_ALGO0);
	    set_reg(&reg, 1, HASH_CR_ALGO1);
            set_reg(&reg, 0, HASH_CR_MODE);
            break;
        case HASH_SHA256:
            set_reg(&reg, 1, HASH_CR_ALGO0);
            set_reg(&reg, 1, HASH_CR_ALGO1);
            set_reg(&reg, 0, HASH_CR_MODE);
            break;
        case HASH_HMAC_SHA1:
            set_reg(&reg, 0, HASH_CR_ALGO0);
            set_reg(&reg, 0, HASH_CR_ALGO1);
            set_reg(&reg, 1, HASH_CR_MODE);
#if CONFIG_USR_DRV_HASH_DEBUG
            printf("hmac procedure not yet supported. Stopping init here\n");
#endif
            goto err;
            break;
        case HASH_HMAC_SHA224:
	    set_reg(&reg, 0, HASH_CR_ALGO0);
	    set_reg(&reg, 1, HASH_CR_ALGO1);
            set_reg(&reg, 1, HASH_CR_MODE);
#if CONFIG_USR_DRV_HASH_DEBUG
            printf("hmac procedure not yet supported. Stopping init here\n");
#endif
            goto err;
            break;
        case HASH_HMAC_SHA256:
            set_reg(&reg, 1, HASH_CR_ALGO0);
            set_reg(&reg, 1, HASH_CR_ALGO1);
            set_reg(&reg, 1, HASH_CR_MODE);
#if CONFIG_USR_DRV_HASH_DEBUG
            printf("hmac procedure not yet supported. Stopping init here\n");
#endif
            goto err;
            break;
        default:
#if CONFIG_USR_DRV_HASH_DEBUG
            printf("unsupported hash algorithm!\n");
#endif
            goto err;
    }
    if (use_dma) {
        set_reg(&reg, 1, HASH_CR_DMAE);
    }

    /* setting CR with all configured fields */
    write_reg_value(_r_CORTEX_M_HASH_CR, reg);


    /* let's configure the interrupts */
    if (use_it) {
        set_reg(_r_CORTEX_M_HASH_IMR, 1, HASH_IMR_DCIE);
    }

    /* end of init, activating HASH */
    set_reg(_r_CORTEX_M_HASH_CR, 1, HASH_CR_INIT);

    return 0;
err:
    return -1;
}

int hash_early_init(hash_transfert_mode_t transfert_mode,
                        hash_map_mode_t       map_mode,
                        hash_dev_mode_t       dev_mode)
{
    e_syscall_ret ret;

    memset((void*)&dev_hash, 0, sizeof(device_t));

    if (transfert_mode != HASH_TRANS_NODMA) {

        dma_hash.dma          = DMA_HASH_CTRL;
        dma_hash.stream       = DMA_STREAM_HASH_IN;
        dma_hash.channel      = DMA_CHANNEL_HASH_IN;
        dma_hash.dir          = MEMORY_TO_PERIPHERAL;
        dma_hash.in_addr      = (physaddr_t) 0;
        dma_hash.out_addr     = (volatile physaddr_t)_r_CORTEX_M_HASH_DIN;
        dma_hash.in_prio      = DMA_PRI_HIGH;
        dma_hash.size         = 0;
        dma_hash.mode         = DMA_DIRECT_MODE;
        dma_hash.mem_inc      = 1;
        dma_hash.dev_inc      = 0;
        dma_hash.datasize     = DMA_DS_WORD;
        dma_hash.mem_burst    = DMA_BURST_SINGLE;
        dma_hash.dev_burst    = DMA_BURST_SINGLE;
        dma_hash.flow_control = DMA_FLOWCTRL_DMA;
        dma_hash.in_handler   = (user_dma_handler_t) dma_hash_handler;
        dma_hash.out_handler  = (user_dma_handler_t) dma_hash_handler;    /* not used */

#if CONFIG_USR_DRV_HASH_DEBUG
        printf("init DMA HASH in ...\n");
#endif

        int dma_hash_desc_ = dma_hash_desc;
        ret = sys_init(INIT_DMA, &dma_hash, (int*)&dma_hash_desc_);
        if (ret != SYS_E_DONE) {
            goto err;
        }
        dma_hash_desc = dma_hash_desc_;
        use_dma = true;
    }

    switch (map_mode) {
        case HASH_MAP_AUTO:
        case HASH_MAP_VOLUNTARY:
            {
#if CONFIG_USR_DRV_HASH_DEBUG
                printf("Declaring hash device\n");
#endif
                const char *name = "hash";
                memset((void*)&dev_hash, 0, sizeof(device_t));
                strncpy((char*)dev_hash.name, name, sizeof (dev_hash.name));
                dev_hash.address = hash_dev_infos.address;
                dev_hash.size = hash_dev_infos.size;
                if (map_mode == HASH_MAP_AUTO) {
                    dev_hash.map_mode = DEV_MAP_AUTO;
                } else {
                    dev_hash.map_mode = DEV_MAP_VOLUNTARY;
                }
                if (dev_mode == HASH_IT_DIGEST_COMPLETE) {
                    dev_hash.irq_num = 1;
                    dev_hash.irqs[0].handler = hash_handler;
                    dev_hash.irqs[0].irq = HASH_IRQ;
                    dev_hash.irqs[0].mode = IRQ_ISR_STANDARD;
                    /* SR register*/
                    dev_hash.irqs[0].posthook.status = 0x00024;
                    dev_hash.irqs[0].posthook.data   = 0;
                    /* read SR */
                    dev_hash.irqs[0].posthook.action[0].instr = IRQ_PH_READ;
                    dev_hash.irqs[0].posthook.action[0].read.offset = 0x0024;
                    /* clear Digest complete status */
                    dev_hash.irqs[0].posthook.action[2].instr = IRQ_PH_WRITE;
                    dev_hash.irqs[0].posthook.action[2].write.offset = 0x0024;
                    dev_hash.irqs[0].posthook.action[2].write.value  = 0x0;
                    dev_hash.irqs[0].posthook.action[2].write.mask   = HASH_SR_DCIS_Msk;
                    use_it = true;
                } else {
                    dev_hash.irq_num = 0;
                }
                dev_hash.gpio_num = 0;
                ret = sys_init(INIT_DEVACCESS, (device_t*)&dev_hash, (int*)&dev_hash_desc);
                if (ret != SYS_E_DONE) {
                    goto err;
                }
                break;
            }
        default:
            {
                printf("invalid map mode!\n");
                goto err;
            }
    }

#if CONFIG_USR_DRV_HASH_DEBUG
    printf("sys_init returns %s !\n", strerror(ret));
#endif
    return 0;
err:
    return -1;
}

int hash_get_digest(uint8_t *digest, uint32_t digest_size, hash_algo_t algo){
    /* Get the algorithm type */
    switch (algo) {
        case HASH_MD5:
	    if(digest_size < 16){
		goto err;
	    }
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(0)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(1)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(2)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(3)));
            break;
        case HASH_SHA1:
        case HASH_HMAC_SHA1:
	    if(digest_size < 20){
		goto err;
	    }
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(0)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(1)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(2)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(3)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(4)));
            break;
        case HASH_SHA224:
        case HASH_HMAC_SHA224:
	    if(digest_size < 28){
		goto err;
	    }
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(0)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(1)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(2)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(3)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(4)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(5)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(6)));
            break;
        case HASH_SHA256:
        case HASH_HMAC_SHA256:
	    if(digest_size < 32){
		goto err;
	    }
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(0)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(1)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(2)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(3)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(4)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(5)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(6)));
            digest += 4;
            *(uint32_t *) digest = htonl(read_reg_value(HASH_HR(7)));
            break;
        default:
            goto err;
    }

    return 0;
err:
    return -1;
}
