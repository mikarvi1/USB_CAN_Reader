/**
**  Simple ROS Node
**/
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <asm/termbits.h> /* struct termios2 */
#include <time.h>
#include <ctype.h>
#include <signal.h>
#include <sys/time.h>
#include <ros/ros.h>

#define CANUSB_INJECT_SLEEP_GAP_DEFAULT 200 /* ms */
#define CANUSB_TTY_BAUD_RATE_DEFAULT 2000000
#define CRC_CCITT_INIT 0xFFFF
#define CRC_CCITT_POLY 0x1021U


typedef enum {
  CANUSB_SPEED_1000000 = 0x01,
  CANUSB_SPEED_800000  = 0x02,
  CANUSB_SPEED_500000  = 0x03,
  CANUSB_SPEED_400000  = 0x04,
  CANUSB_SPEED_250000  = 0x05,
  CANUSB_SPEED_200000  = 0x06,
  CANUSB_SPEED_125000  = 0x07,
  CANUSB_SPEED_100000  = 0x08,
  CANUSB_SPEED_50000   = 0x09,
  CANUSB_SPEED_20000   = 0x0a,
  CANUSB_SPEED_10000   = 0x0b,
  CANUSB_SPEED_5000    = 0x0c,
  CANUSB_SPEED_0 = 0x0,
} CANUSB_SPEED;

typedef enum {
  CANUSB_MODE_NORMAL          = 0x00,
  CANUSB_MODE_LOOPBACK        = 0x01,
  CANUSB_MODE_SILENT          = 0x02,
  CANUSB_MODE_LOOPBACK_SILENT = 0x03,
} CANUSB_MODE;

typedef enum {
  CANUSB_FRAME_STANDARD = 0x01,
  CANUSB_FRAME_EXTENDED = 0x02,
} CANUSB_FRAME;

typedef enum {
    CANUSB_INJECT_PAYLOAD_MODE_RANDOM      = 0,
    CANUSB_INJECT_PAYLOAD_MODE_INCREMENTAL = 1,
    CANUSB_INJECT_PAYLOAD_MODE_FIXED       = 2,
} CANUSB_PAYLOAD_MODE;

static int tty_fd;
static int publisher_not_timedout = 1;
static int terminate_after = 100;
static int program_running = 1;
static int inject_payload_mode = CANUSB_INJECT_PAYLOAD_MODE_FIXED;
static float inject_sleep_gap = CANUSB_INJECT_SLEEP_GAP_DEFAULT;
static int print_traffic = 0;
static int frame_start_found = 0;
static unsigned char previous_byte = 0xfd;
//used to publish the status for other node
static  ros::NodeHandle *p_n;
void CCITT_CRC16Init(const unsigned char* bytes, int len);
unsigned short CCITT_CRC16;
struct frame_buffer {
        char bframe[8];
};



static CANUSB_SPEED canusb_int_to_speed(int speed)
{
  switch (speed) {
  case 1000000:
    return CANUSB_SPEED_1000000;
  case 800000:
    return CANUSB_SPEED_800000;
  case 500000:
    return CANUSB_SPEED_500000;
  case 400000:
    return CANUSB_SPEED_400000;
  case 250000:
    return CANUSB_SPEED_250000;
  case 200000:
    return CANUSB_SPEED_200000;
  case 125000:
    return CANUSB_SPEED_125000;
  case 100000:
    return CANUSB_SPEED_100000;
  case 50000:
    return CANUSB_SPEED_50000;
  case 20000:
    return CANUSB_SPEED_20000;
  case 10000:
    return CANUSB_SPEED_10000;
  case 5000:
    return CANUSB_SPEED_5000;
  default:
    return (CANUSB_SPEED)0;
  }
}



static int generate_checksum(const unsigned char *data, int data_len)
{
  int i, checksum;

  checksum = 0;
  for (i = 0; i < data_len; i++) {
    checksum += data[i];
  }

  return checksum & 0xff;
}



static int frame_is_complete(const unsigned char *frame, int frame_len)
{
  if (frame_len > 0) {
    if (frame[0] != 0xaa) {
      /* Need to sync on 0xaa at start of frames, so just skip. */
      return 1;
    }
  }

  if (frame_len < 2) {
    return 0;
  }

  if (frame[1] == 0x55) { /* Command frame... */
    if (frame_len >= 20) { /* ...always 20 bytes. */
      return 1;
    } else {
      return 0;
    }
  } else if ((frame[1] >> 4) == 0xc) { /* Data frame... */
    if (frame_len >= (frame[1] & 0xf) + 5) { /* ...payload and 5 bytes. */
      return 1;
    } else {
      return 0;
    }
  }

  /* Unhandled frame type. */
  return 1;
}


static int frame_send(int tty_fd, const unsigned char *frame, int frame_len)
{
  int result, i;

  if (print_traffic) {
    printf(">>> ");
    for (i = 0; i < frame_len; i++) {
      printf("%02x ", frame[i]);
    }
    if (print_traffic > 1) {
      printf("    '");
      for (i = 4; i < frame_len - 1; i++) {
        printf("%c", isalnum(frame[i]) ? frame[i] : '.');
      }
      printf("'");
    }
    printf("\n");
  }

  result = write(tty_fd, frame, frame_len);
  if (result == -1) {
    fprintf(stderr, "write() failed: %s\n", strerror(errno));
    return -1;
  }

  return frame_len;
}



static int frame_recv(int tty_fd, unsigned char *frame, int frame_len_max)
{
  int result, frame_len, checksum;
  unsigned char byte;

  if (print_traffic)
    fprintf(stderr, "<<< ");

  frame_len = 0;
  while (program_running) {
    result = read(tty_fd, &byte, 1);
    if (result == -1) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        fprintf(stderr, "read() failed: %s\n", strerror(errno));
        return -1;
      }

    } else if (result > 0) {
      if (print_traffic)
        fprintf(stderr, "%02x ", byte);

      if (frame_len == frame_len_max) {
        fprintf(stderr, "frame_recv() failed: Overflow\n");
        return -1;
      }

      frame[frame_len++] = byte;

      if (frame_is_complete(frame, frame_len)) {
        break;
      }
    }

    usleep(10);
  }

  if (print_traffic)
    fprintf(stderr, "\n");

  /* Compare checksum for command frames only. */
  if ((frame_len == 20) && (frame[0] == 0xaa) && (frame[1] == 0x55)) {
    checksum = generate_checksum(&frame[2], 17);
    if (checksum != frame[frame_len - 1]) {
      fprintf(stderr, "frame_recv() failed: Checksum incorrect\n");
      return -1;
    }
  }

  return frame_len;
}



static int command_settings(int tty_fd, CANUSB_SPEED speed, CANUSB_MODE mode, CANUSB_FRAME frame)
{
  int cmd_frame_len;
  unsigned char cmd_frame[20];

  cmd_frame_len = 0;
  cmd_frame[cmd_frame_len++] = 0xaa;
  cmd_frame[cmd_frame_len++] = 0x55;
  cmd_frame[cmd_frame_len++] = 0x12;
  cmd_frame[cmd_frame_len++] = speed;
  cmd_frame[cmd_frame_len++] = frame;
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Filter ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = 0; /* Mask ID not handled. */
  cmd_frame[cmd_frame_len++] = mode;
  cmd_frame[cmd_frame_len++] = 0x01;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = 0;
  cmd_frame[cmd_frame_len++] = generate_checksum(&cmd_frame[2], 17);

  if (frame_send(tty_fd, cmd_frame, cmd_frame_len) < 0) {
    return -1;
  }

  return 0;
}


static int hex_value(int c)
{
  if (c >= 0x30 && c <= 0x39) /* '0' - '9' */
    return c - 0x30;
  else if (c >= 0x41 && c <= 0x46) /* 'A' - 'F' */
    return (c - 0x41) + 10;
  else if (c >= 0x61 && c <= 0x66) /* 'a' - 'f' */
    return (c - 0x61) + 10;
  else
    return -1;
}



static int convert_from_hex(const char *hex_string, unsigned char *bin_string, int bin_string_len)
{
  int n1, n2, high;

  high = -1;
  n1 = n2 = 0;
  while (hex_string[n1] != '\0') {
    if (hex_value(hex_string[n1]) >= 0) {
      if (high == -1) {
        high = hex_string[n1];
      } else {
        bin_string[n2] = hex_value(high) * 16 + hex_value(hex_string[n1]);
        n2++;
        if (n2 >= bin_string_len) {
          printf("hex string truncated to %d bytes\n", n2);
          break;
        }
        high = -1;
      }
    }
    n1++;
  }

  return n2;
}

#define CAN_OFFSET_BYTE 0x0

static void dump_data_frames(int tty_fd)
{
  int i, frame_len;
  int run_cnt =0;
  unsigned char frame[32];
  struct timespec ts;
  static int start_track=0;
  static int skip_frame=0;
  
  frame_len = frame_recv(tty_fd, frame, sizeof(frame));
  clock_gettime(CLOCK_MONOTONIC, &ts);
  printf("%lu.%06lu ", ts.tv_sec, ts.tv_nsec / 1000);

  if (frame_len == -1) {
  	ROS_INFO("Haystack Battery Monitor error!!!");

  } 
  else 
  {
  	if (run_cnt > 100) {
  		ROS_INFO("Haystack Battery Monitor run cnt hit!!!");
		return;
	}

     	if ((frame_len >= 6) && (frame[0] == 0xaa) &&
          		((frame[1] >> 4) == 0xc)) 
	{
     		ROS_INFO("Frame ID: %02x%02x, Data: ", frame[3], frame[2]);
        
		if (frame[3] == 0x03 && frame[2] == 0xAA)
		{
			ROS_WARN("Connection status %02x", frame[frame_len-2]);
			if (frame[frame_len-2] == 4)
	    			p_n->setParam("/haystack/battery_status", "CHARGING");
			else
	    			p_n->setParam("/haystack/battery_status", "DISCHARGING");
	
			publisher_not_timedout = 0;
		}
		if (frame[3] == 0x02 && frame[2] == 0xAA)
		{
	    		ROS_WARN("SOC %02x", frame[frame_len-9]);
	    		ROS_WARN("SOH %02x", frame[frame_len-2]);
	    		p_n->setParam("/haystack/battery_percentage", frame[frame_len-9]);
	    		publisher_not_timedout = 0;
		}
		for (i = frame_len - 2; i > 3; i--) {
          		ROS_INFO("%02x ", frame[i]);
        	}
        	ROS_INFO("\n");

      } 
	 else 
	 {
    		//printf("%lu.%06lu ", ts.tv_sec, ts.tv_nsec / 1000);
        	//printf("UN: 0x%02x l=%d ",frame[CAN_OFFSET_BYTE],frame_len);
        	//for (i = frame_len; i > 0; i--) {
        	//  printf("[%d]%02x ",i, frame[i]);
		//}
		//printf("\n");
		if ((frame[CAN_OFFSET_BYTE]==0xff)&&(start_track == 0)) 
		{
			//reset/start the track
			start_track=1;
			skip_frame=14;
			p_n->setParam("/haystack/battery_status", "DISCHARGING");
			ROS_INFO("\n\n>T1 0x%02x, %d\n\n", frame[CAN_OFFSET_BYTE], frame[CAN_OFFSET_BYTE]);
		} else 
		if (((start_track == 0x7) || (start_track == 0x1)) && (frame[CAN_OFFSET_BYTE]==0x00)) 
		{
			//if not the first byte tracking skip frame
			if (start_track == 0x7)
				start_track |= 0x8;
			else
				start_track |= 0x2;
		} 
		else 
		if (frame[CAN_OFFSET_BYTE] == 0x3d && start_track == 0x3) 
		{
			start_track |= 0x4;
		} 
		else 
		if (start_track == 0xf) {
			skip_frame--;
			if (skip_frame == 0) {
			//Stop tracking and print/publish battery
				start_track = 0;
				ROS_INFO("\n\n>>>>>>Battery percentage is 0x%02x, %d\n\n", frame[CAN_OFFSET_BYTE], frame[CAN_OFFSET_BYTE]);
					//publish battery percentage
				ROS_WARN("SOC %02x", frame[CAN_OFFSET_BYTE]);
	    			p_n->setParam("/haystack/battery_percentage", frame[CAN_OFFSET_BYTE]);
			}
		} 
		else 
		{
			start_track = 0;
		}


	
        	for (i = 0; i <= frame_len; i++) {
          		ROS_INFO("%02x ", frame[i]);
        	}
        	ROS_INFO("\n");
      }
    }

}




static int US_battery_frame_is_complete(unsigned char* frame, int* frame_len,int tty_fd)
{
    
    int ret;
    unsigned char ch;
    if (*frame_len > 0) 
    {
        if (frame[0] != 0xaa) // && previous_byte != 0x55) 
	    {
            /* Need to sync on 0xaa at start of frames, so just skip. */
            return 1;
        }
    }


    if (frame[*frame_len - 1] == 0x55)
    {
	    ret = read(tty_fd, &ch, 1);
        if (ret == -1)
	    {
		    if (errno != EAGAIN && errno != EWOULDBLOCK) 
		    {
                return -1;
            }
           printf("waiting for frame_is_complete %d %d %d\n", ret,tty_fd,errno);
	       usleep(10);
		   
           return 0;
           
	    }
        //rewind(fd,1);
        //fseek(pFile, -sz, SEEK_CUR)
        if (ch == 0xaa)
        {
            frame_start_found = 1;
            return 1;
        }
        else
        {
            frame[*frame_len] == ch;
            *frame_len = *frame_len + 1;
            return 0;
        }
    }
    else return 0;

}


static int US_battery_frame_recv(int tty_fd, unsigned char* frame, int frame_len_max)
{
    	int result, frame_len, checksum;
    	unsigned char byte;
    	int ret;
    	frame_len = 0;

    	while (program_running) 
    	{
        	result = 0;
        	if (frame_start_found == 2)
        	{
            	frame_start_found = 0;
            	frame_len = 1;
           	frame[0] = 0xaa;
        	}

        	if (frame_start_found == 0)
        	{
            	result = read(tty_fd,&byte,1);
           	if (previous_byte != 0xfd && byte == 0xaa && previous_byte != 0x55)
            	{
                	if ((result == 1) && (byte == 0xaa) && (previous_byte == 0xaa))
                	{
                    		//result = fread(&byte, sizeof(byte), 1, tty_fd);
                    		continue;

                	}
                	result = read(tty_fd, &byte, 1);
                	if ((result == 1) && (byte == 0xaa))
                	{
                    		//result = fread(&byte, sizeof(byte), 1, tty_fd);
                    		continue;

                	}
                	result = read(tty_fd, &byte, 1);
                	if ((result == 1) && byte == 0xaa)
                	{
                    //result = fread(&byte, sizeof(byte), 1, tty_fd);
                    		continue;

                	}
                	result = read(tty_fd, &byte, 1);
                	result = read(tty_fd, &byte, 1);
           	    }
        	}
        	if(frame_start_found == 1)
        	{
            	frame_start_found = 0;
            	frame_len = 0;
            	frame[0] = 0xaa;
		 	//memset(frame,0,32);
           	byte = 0xaa;
           	previous_byte = 0x55;
           	result = 100;
        	}
        	if (result == -1) 
	   	{
            	if (errno != EAGAIN && errno != EWOULDBLOCK) 
		 	{
                	fprintf(stderr, "read() failed: %d\n", (errno));
                	return -1;
            	}

         	}
         	else
	    	{
            	if (result > 0) 
		 	{
                	if (print_traffic)
                    		fprintf(stderr, "%02x\n ", byte);
                	if (frame_len == frame_len_max) 
				{
                    		fprintf(stderr, "frame_recv() failed: Overflow\n");
                    		return -1;
                	}

                	if ((previous_byte == 0x55) && (byte == 0xaa) && (result != 100))
                	{
                    		int temp;
                    		temp = frame_len;
                    		frame_len = 0;
                    		frame_start_found = 2;
                    		previous_byte = 0xaa;
                    		return temp;
                	}
				else
				{
               		    frame[frame_len++] = byte;

                	    if (frame_len == frame_len_max) 
				    {
                    	    	fprintf(stderr, "frame_recv() failed: Overflow\n");
                    	    	return -1;
                	    }

               		    ret = US_battery_frame_is_complete(frame, &frame_len, tty_fd);
                
		        	    if (ret == 1)
                	    {
					     previous_byte = byte;
                    	    		break;
                	    }
                	    if (ret < 0)
                	    {
                    	    		printf("Frame Complete Error!!%d \n",frame_len);
		       	    	    	return -1;
                	    }
                	    previous_byte = byte;
                        //usleep(10);
            	    }

        	    	}
		    }
		    usleep(10);
	    }
	    return frame_len;
}

struct frame_buffer frameBuffer[32];
int multiframelen = 0;
int frameComplete = 0;
//int framestarted = false;
int dontprocess;

static void US_battery_dump_data_frames(int tty_fd)
{
    int i, frame_len;
    unsigned char frame[32];
    //struct frame_buffer frameBuffer[32];
    //int multiframelen = 0;
    //int frameComplete = 0;
    struct timespec ts;
    //int framestarted = false;
    //int dontprocess;
    //unsigned char byte;

    //while (program_running) {
        dontprocess = false;
        frame_len = US_battery_frame_recv(tty_fd, frame, sizeof(frame));

        clock_gettime(CLOCK_MONOTONIC, &ts);
        
        if (frame_len == -1) {
            printf("Frame recieve error!\n");
		 return; 
            //break;
        }
	
        printf("%lu.%06lu ", ts.tv_sec, ts.tv_nsec / 1000);
        
	   /*
	   for (int i = 0;i < frame_len;i++)
        {
            printf("%x ", frame[i]);
        }
        printf("Frame length %d\n", frame_len);
	   */
        
	   if (frame_len > 8 && frame_len < 16)
        {
            if (multiframelen > 31)
                multiframelen = 0;
            if (frame_len > 7 && frame_len != 15)
            {
                if (frame[frame_len - 2] != 0x67)
                {
                    //wrong frame
                    dontprocess = true;

                }
			else //usleep(100);  //Sleep for 100 msec for next set of frames.
			{
			}
            }
            if (dontprocess == false)
            {
                if ((frame_len > 13) && (frame[13] == 0x80))
                {
                    multiframelen = 0;
                    memset(frameBuffer, 0, sizeof(frameBuffer));
                }
                memcpy(frameBuffer[multiframelen].bframe, frame + 6, (frame_len  - 7));

                if ((frameBuffer[0].bframe[7] == 0x80) && frameBuffer[multiframelen].bframe[(frame_len - 2) - 6] == 0x67)
                {
                    frameComplete = 1;
                    unsigned char data1[255];
                    unsigned char* pointertodata1 = data1;
                    unsigned short receivedCRC = 0;

                    for (int j = 0;j < multiframelen;j++)
                    {
                        if (j == 0)
                        {
                            memcpy(pointertodata1, frameBuffer[j].bframe + 2, 6);
                            pointertodata1 = pointertodata1 + 6;
                        }
                        else
                        {
                            memcpy(pointertodata1, frameBuffer[j].bframe, 8);
                            pointertodata1 = pointertodata1 + 8;
                        }
                        //printf(" data %x\n", data1);

                    }
                    memcpy(pointertodata1, frameBuffer[multiframelen].bframe, frame_len - 7);

                    for (int i = 0; i < ((multiframelen * 8 + frame_len - 6) - 2); i++)
                        printf(" %x", data1[i]);
                    printf("received data\n");

                    CCITT_CRC16Init(data1, ((multiframelen * 8 + frame_len - 6) - 2));
                    
                    
                    receivedCRC = (frameBuffer[0].bframe[1] << 8) + (frameBuffer[0].bframe[0]);
                    printf("battery %x\n", frameBuffer[1].bframe[5]);


				ROS_INFO("\n\n>>>>>>Battery percentage is 0x%02x, %d\n\n", frameBuffer[1].bframe[5], frameBuffer[1].bframe[5]);
					//publish battery percentage
				ROS_WARN("SOC %02x", frameBuffer[1].bframe[5]);
	    			p_n->setParam("/haystack/battery_percentage", frameBuffer[1].bframe[5]);


				ROS_WARN("Connection status %02x", frameBuffer[2].bframe[2]);
				if (frameBuffer[2].bframe[2] > 0)
	    				p_n->setParam("/haystack/battery_status", "CHARGING");
				else
	    				p_n->setParam("/haystack/battery_status", "DISCHARGING");
	
				publisher_not_timedout = 0;
	

                    if (CCITT_CRC16 == receivedCRC)
                    {
                        printf("Valid Frame\n");
                    }
                    else
                    {
                        printf("Received CRC %x\n", receivedCRC);
                        printf("Calculated CRC %x\n", CCITT_CRC16);
                    }
                }
                else 
                	multiframelen++;
            }
        }
        
        if (frameComplete == 1)
        {
            frameComplete = 0;
            
            for (int j = 0;j <= multiframelen;j++)
            {
                for (int i = 0;i < 8;i++)
                    printf(" %x ", frameBuffer[j].bframe[i]);
                printf("\n");
            }
            printf("\n");
            multiframelen = 0;
            memset(frameBuffer, 0, sizeof(frameBuffer));
        }
        
    
    //}

    if (terminate_after && (--terminate_after == 0))
    {
        program_running = 0;
    }
}



void CCITT_CRCStep(unsigned char byte)
{
    unsigned short j;
    CCITT_CRC16 ^= ((unsigned short)byte << 8);
    for (j = 0; j < 8; j++)
    {
        CCITT_CRC16 = (CCITT_CRC16 & 0x8000U) ? ((CCITT_CRC16 << 1) ^ CRC_CCITT_POLY) : (CCITT_CRC16 << 1);
    }
}


void CCITT_CRC_ARRAY(const unsigned char* bytes, int len)
{
    while (len--) CCITT_CRCStep(*bytes++);
}


void CCITT_CRC16Init(const unsigned char* bytes, int len)
{
    CCITT_CRC16 = CRC_CCITT_INIT;
    CCITT_CRC_ARRAY(bytes, len);
}


static int adapter_init(const char *tty_device, int baudrate)
{
  int tty_fd, result;
  struct termios2 tio;

  tty_fd = open(tty_device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (tty_fd == -1) {
    fprintf(stderr, "open(%s) failed: %s\n", tty_device, strerror(errno));
    return -1;
  }

  result = ioctl(tty_fd, TCGETS2, &tio);
  if (result == -1) {
    fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
    close(tty_fd);
    return -1;
  }

  tio.c_cflag &= ~CBAUD;
  tio.c_cflag = BOTHER | CS8 | CSTOPB;
  tio.c_iflag = IGNPAR;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_ispeed = baudrate;
  tio.c_ospeed = baudrate;

  result = ioctl(tty_fd, TCSETS2, &tio);
  if (result == -1) {
    fprintf(stderr, "ioctl() failed: %s\n", strerror(errno));
    close(tty_fd);
    return -1;
  }

  return tty_fd;
}



static void display_help(const char *progname)
{
  fprintf(stderr, "Usage: %s <options>\n", progname);
  fprintf(stderr, "Options:\n"
     "  -h          Display this help and exit.\n"
     "  -t          Print TTY/serial traffic debugging info on stderr.\n"
     "  -d DEVICE   Use TTY DEVICE.\n"
     "  -s SPEED    Set CAN SPEED in bps.\n"
     "  -b BAUDRATE Set TTY/serial BAUDRATE (default: %d).\n"
     "  -i ID       Inject using ID (specified as hex string).\n"
     "  -j DATA     CAN DATA to inject (specified as hex string).\n"
     "  -n COUNT    Terminate after COUNT frames (default: infinite).\n"
     "  -g MS       Inject sleep gap in MS milliseconds (default: %d ms).\n"
     "  -m MODE     Inject payload MODE (%d = random, %d = incremental, %d = fixed).\n"
     "\n",
     CANUSB_TTY_BAUD_RATE_DEFAULT,
     CANUSB_INJECT_SLEEP_GAP_DEFAULT,
     CANUSB_INJECT_PAYLOAD_MODE_RANDOM,
     CANUSB_INJECT_PAYLOAD_MODE_INCREMENTAL,
     CANUSB_INJECT_PAYLOAD_MODE_FIXED);
}

//int pub_soc_timeout(const ros::TimerEvent & event)
int pub_soc_timeout(void)
{
	int i=0;
      ROS_WARN("Haystack Battery Monitor timed hit!!!");
	for (i=0;i<30;i++) {
     		//dump_data_frames(tty_fd);
		US_battery_dump_data_frames(tty_fd);
	}
}


int main(int argc, char *argv[])
{
  int c,  interval = 3;
  char tty_device[] = "/dev/battery", *inject_data = NULL, *inject_id = NULL;
  CANUSB_SPEED speed = CANUSB_SPEED_500000;
  int baudrate = CANUSB_TTY_BAUD_RATE_DEFAULT;
  ros::init(argc, argv, "battery_can");
  // Create a ROS node handle
  ros::NodeHandle n;
  ros::Rate loop_rate(2);

  p_n = &n;
  speed = CANUSB_SPEED_500000;


  if (tty_device == NULL) {
    fprintf(stderr, "Please specify a TTY!\n");
    ROS_ERROR("Haystack Battery Monitor TTY not found");
    display_help(argv[0]);
    return EXIT_FAILURE;
  }

  if (speed == 0) {
    ROS_ERROR("Haystack Battery Monitor please check connection or contact developer!");
    display_help(argv[0]);
    return EXIT_FAILURE;
  }

  tty_fd = adapter_init(tty_device, baudrate);
  if (tty_fd == -1) {
    ROS_ERROR("Haystack Battery Monitor TTY not opened!!");
    return EXIT_FAILURE;
  }
  command_settings(tty_fd, speed, CANUSB_MODE_NORMAL, CANUSB_FRAME_STANDARD);
 
  while (ros::ok())
  { 
  	pub_soc_timeout();
  	ROS_INFO("Haystack Battery Monitor runing!!!");
  	loop_rate.sleep();
  	ros::spinOnce();
  }
  return EXIT_SUCCESS;
}
