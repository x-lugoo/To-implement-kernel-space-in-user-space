#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>

#include "hw-addresses.h"



#define PAGE_SIZE 4096

#define PA//-------- Relative offsets for DMA registers
//DMA Channel register sets (format of these registers is found in DmaChannelHeader struct):
#define DMACH(n) (0x100*(n))
//Each DMA channel has some associated registers, but only CS (control and status), CONBLK_AD (control block address), and DEBUG are writeable
//DMA is started by writing address of the first Control Block to the DMA channel's CONBLK_AD register and then setting the ACTIVE bit inside the CS register (bit 0)
//Note: DMA channels are connected directly to peripherals, so physical addresses should be used (affects control block's SOURCE, DEST and NEXTCONBK addresses).
#define DMAENABLE 0x00000ff0 //bit 0 should be set to 1 to enable channel 0. bit 1 enables channel 1, etc.

//flags used in the DmaChannelHeader struct:
#define DMA_CS_RESET (1 << 31)
#define DMA_CS_ACTIVE (1 << 0)

#define DMA_DEBUG_READ_ERROR (1 << 2)
#define DMA_DEBUG_FIFO_ERROR (1 << 1)
#define DMA_DEBUG_READ_LAST_NOT_SET_ERROR (1 << 0)

//flags used in the DmaControlBlock struct:
#define DMA_CB_TI_DEST_INC (1 << 4)
#define DMA_CB_TI_SRC_INC (1 << 8)

void write_bitmasked(volatile uint32_t *dest, uint32_t mask, uint32_t value)
{

    uint32_t cur = *dest;
    uint32_t new = (cur & (~mask)) | (value & mask);
    *dest = new;
    *dest = new; //added safety for when crossing memory barriers.
}


struct dma_channel_header {
    uint32_t CS; //Control and Status
        //31    RESET; set to 1 to reset DMA
        //30    ABORT; set to 1 to abort current DMA control block (next one will be loaded & continue)
        //29    DISDEBUG; set to 1 and DMA won't be paused when debug signal is sent
        //28    WAIT_FOR_OUTSTANDING_WRITES; set to 1 and DMA will wait until peripheral says all writes have gone through before loading next CB
        //24-27 reserved
        //20-23 PANIC_PRIORITY; 0 is lowest priority
        //16-19 PRIORITY; bus scheduling priority. 0 is lowest
        //9-15  reserved
        //8     ERROR; read as 1 when error is encountered. error can be found in DEBUG register.
        //7     reserved
        //6     WAITING_FOR_OUTSTANDING_WRITES; read as 1 when waiting for outstanding writes
        //5     DREQ_STOPS_DMA; read as 1 if DREQ is currently preventing DMA
        //4     PAUSED; read as 1 if DMA is paused
        //3     DREQ; copy of the data request signal from the peripheral, if DREQ is enabled. reads as 1 if data is being requested, else 0
        //2     INT; set when current CB ends and its INTEN=1. Write a 1 to this register to clear it
        //1     END; set when the transfer defined by current CB is complete. Write 1 to clear.
        //0     ACTIVE; write 1 to activate DMA (load the CB before hand)
    uint32_t CONBLK_AD; //Control Block Address
    uint32_t TI; //transfer information; see DmaControlBlock.TI for description
    uint32_t SOURCE_AD; //Source address
    uint32_t DEST_AD; //Destination address
    uint32_t TXFR_LEN; //transfer length.
    uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
    uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t DEBUG; //controls debug settings
};

struct dma_control_block {
    uint32_t TI; //transfer information
        //31:27 unused
        //26    NO_WIDE_BURSTS
        //21:25 WAITS; number of cycles to wait between each DMA read/write operation
        //16:20 PERMAP; peripheral number to be used for DREQ signal (pacing). set to 0 for unpaced DMA.
        //12:15 BURST_LENGTH
        //11    SRC_IGNORE; set to 1 to not perform reads. Used to manually fill caches
        //10    SRC_DREQ; set to 1 to have the DREQ from PERMAP gate requests.
        //9     SRC_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
        //8     SRC_INC;   set to 1 to automatically increment the source address after each read (you'll want this if you're copying a range of memory)
        //7     DEST_IGNORE; set to 1 to not perform writes.
        //6     DEST_DREG; set to 1 to have the DREQ from PERMAP gate *writes*
        //5     DEST_WIDTH; set to 1 for 128-bit moves, 0 for 32-bit moves
        //4     DEST_INC;   set to 1 to automatically increment the destination address after each read (Tyou'll want this if you're copying a range of memory)
        //3     WAIT_RESP; make DMA wait for a response from the peripheral during each write. Ensures multiple writes don't get stacked in the pipeline
        //2     unused (0)
        //1     TDMODE; set to 1 to enable 2D mode
        //0     INTEN;  set to 1 to generate an interrupt upon completion
    uint32_t SOURCE_AD; //Source address
    uint32_t DEST_AD; //Destination address
    uint32_t TXFR_LEN; //transfer length.
    uint32_t STRIDE; //2D Mode Stride. Only used if TI.TDMODE = 1
    uint32_t NEXTCONBK; //Next control block. Must be 256-bit aligned (32 bytes; 8 words)
    uint32_t _reserved[2];
};


int  make_virt_phys_page(void **virtaddr, void **physaddr)
{
	uint64_t page_info;
	int fd;
	*virtaddr = valloc(PAGE_SIZE); /*valloc memory align with page*/

	if (*virtaddr == NULL) {
		perror("valloc");
		return -1;
	}
	
    	memset(*virtaddr, 0, PAGE_SIZE); /*get the truely phys addrs, this is the key */
	fd = open("/proc/self/pagemap", O_RDONLY);
	if (fd < 0) {
		perror("/proc/self/pagemap");
		return -1;
	}
	lseek(fd, ((size_t)*virtaddr) / PAGE_SIZE * 8, SEEK_SET);
	read(fd, &page_info, 8);
	close(fd);

	*physaddr = (void*)(size_t)(page_info * PAGE_SIZE);
	printf("viraddr %p physaddr 0x%x %s(%d)\n", *virtaddr, *physaddr, __func__, __LINE__);

	return 0;
}

void free_virt_phys_page(void **virtaddr)
{
	free(virtaddr);
}

uint32_t* map_phys_addr(int memfd, int addr)
{
    void *mapped = mmap(NULL, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, memfd, addr);

    if (mapped == MAP_FAILED) {
        printf("failed to map memory (did you remember to run as root?)\n");
        exit(1);
    } else {
        printf("phy addr 0x%x mapped virtaddr: %p\n", addr, mapped);
    }
    return (uint32_t*)mapped;
}

		
int main(void)
{
    int dma_chnum = 5;
    int memfd;
    uint32_t *dma_base_vaddr;
    void *virt_src_page, *phys_src_page;
    void *virt_dest_page, *phys_dest_page;
    void *virt_cb_page, *phys_cb_page;
    char *src_array;
    char str[] = "hello jeff";
    struct dma_control_block *cb;


     memfd = open("/dev/mem", O_RDWR | O_SYNC);

     if (memfd < 0) {
	perror("/dev/mem");
        exit(1);
    }

    dma_base_vaddr = map_phys_addr(memfd, DMA_BASE);
    
    make_virt_phys_page(&virt_src_page, &phys_src_page);
    make_virt_phys_page(&virt_dest_page, &phys_dest_page);
    make_virt_phys_page(&virt_cb_page, &phys_cb_page);
    
    src_array = (char*)virt_src_page;
    
    printf("sizeof str %d %s(%d)\n", sizeof(str), __func__, __LINE__);
    memcpy(src_array, str, sizeof(str));
    printf("%s(%d)\n",__func__, __LINE__);
    cb = (struct dma_control_block*)virt_cb_page;
    
    
    cb->TI = DMA_CB_TI_SRC_INC | DMA_CB_TI_DEST_INC; //after each byte copied, we want to increment the source and destination address of the copy, otherwise we'll be copying to the same address.
    cb->SOURCE_AD = (uint32_t)phys_src_page; //set source and destination DMA address
    cb->DEST_AD = (uint32_t)phys_dest_page;
    cb->TXFR_LEN = sizeof(str); 
    cb->STRIDE = 0; //no 2D stride
    cb->NEXTCONBK = 0; //no next control block
    
    write_bitmasked(dma_base_vaddr + DMAENABLE / 4, 1 << dma_chnum, 1 << dma_chnum);
    
    struct dma_channel_header *dma_header = (struct dma_channel_header*)(dma_base_vaddr + (DMACH(dma_chnum)) / 4);
    dma_header->CS = DMA_CS_RESET; //make sure to disable dma first.

    sleep(1); //give time for the reset command to be handled.

    dma_header->DEBUG = DMA_DEBUG_READ_ERROR | DMA_DEBUG_FIFO_ERROR | DMA_DEBUG_READ_LAST_NOT_SET_ERROR; // clear debug error flags
    dma_header->CONBLK_AD = (uint32_t)phys_cb_page; //we have to point it to the PHYSICAL address of the control block (cb1)
    dma_header->CS = DMA_CS_ACTIVE; //set active bit, but everything else is 0.
    
    sleep(1); //give time for copy to happen
    
    printf("%s(%d)\n",__func__, __LINE__);
    printf("src string: '%s'\n", (char*)virt_src_page);
    printf("destination reads: '%s'\n", (char*)virt_dest_page);
    
    //cleanup
    free_virt_phys_page(virt_cb_page);
    free_virt_phys_page(virt_dest_page);
    free_virt_phys_page(virt_src_page);
    return 0;
}

