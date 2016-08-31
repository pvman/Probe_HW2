#ifndef FIFO__H
#define FIFO__H
 

 
//size: 4,8,16,32...128
#define FIFO( size )\
  struct {\
    unsigned short buf[size];\
    unsigned short tail;\
    unsigned short head;\
  } 
 
//FIFO(1024)		SPI_to_FIFO_to_USBtx;

	
//the number of elements in the queue
#define FIFO_COUNT(fifo)     (fifo.head-fifo.tail)
 
//the size fifo
#define FIFO_SIZE(fifo)      (sizeof(fifo.buf)/sizeof(fifo.buf[0]))
 
//fifo filled?
#define FIFO_IS_FULL(fifo)   (FIFO_COUNT(fifo)==FIFO_SIZE(fifo))
 
//fifo empty?
#define FIFO_IS_EMPTY(fifo)  (fifo.tail==fifo.head)
 
//free space in fifo
#define FIFO_SPACE(fifo)     (FIFO_SIZE(fifo)-FIFO_COUNT(fifo))
 
//place to fifo
#define FIFO_PUSH(fifo, byte) \
  {\
    fifo.buf[fifo.head & (FIFO_SIZE(fifo)-1)]=byte;\
    fifo.head++;\
  }
 
//take the first element of the fifo
#define FIFO_FRONT(fifo) (fifo.buf[fifo.tail & (FIFO_SIZE(fifo)-1)])
 
//decrement the number of elements in the queue
#define FIFO_POP(fifo)   \
  {\
      fifo.tail++; \
  }
 
//clean fifo
#define FIFO_FLUSH(fifo)   \
  {\
    fifo.tail=0;\
    fifo.head=0;\
  } 
 
	//*==========================================
#define FIFO8( size )\
  struct {\
    unsigned char buf[size];\
    unsigned short tail;\
    unsigned short head;\
  } 	
	
	//FIFO8(256)		USBrx_to_FIFO_to_SPI;	
	
	
//the number of elements in the queue
#define FIFO8_COUNT(fifo)     (fifo.head-fifo.tail)
 
//the size fifo
#define FIFO8_SIZE(fifo)      (sizeof(fifo.buf)/sizeof(fifo.buf[0]))
 
//fifo filled?
#define FIFO8_IS_FULL(fifo)   (FIFO8_COUNT(fifo)==FIFO8_SIZE(fifo))
 
//fifo empty?
#define FIFO8_IS_EMPTY(fifo)  (fifo.tail==fifo.head)
 
//free space in fifo
#define FIFO8_SPACE(fifo)     (FIFO8_SIZE(fifo)-FIFO8_COUNT(fifo))
 
//place to fifo
#define FIFO8_PUSH(fifo, byte) \
  {\
    fifo.buf[fifo.head & (FIFO8_SIZE(fifo)-1)]=byte;\
    fifo.head++;\
  }
 
//take the first element of the fifo
#define FIFO8_FRONT(fifo) (fifo.buf[fifo.tail & (FIFO8_SIZE(fifo)-1)])
 
//decrement the number of elements in the queue
#define FIFO8_POP(fifo)   \
  {\
      fifo.tail++; \
  }
 
//clean fifo
#define FIFO8_FLUSH(fifo)   \
  {\
    fifo.tail=0;\
    fifo.head=0;\
  } 
	
	
	
	
#endif //FIFO__H
	/*   */
	