//Nathan Noronha
//UFID:76631512

#include <linux/ioctl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/wait.h>
#include "pthread.h"
#include <time.h>
#include <sys/mman.h>
#include <stdbool.h>

#define PIPE_WRITE 1
#define PIPE_READ 0
#define CAPSLOCK_OFF 0
#define CAPSLOCK_ON 1
#define EVENT_SIZE 1000


typedef struct thread_signal
{ 
  pthread_mutex_t mutex;
  int cond;
} thread_signal_t;


// This structure is used to transfer the led commands from  driver simulation to the keyboard process using mmap.
typedef struct ledsBuf
{
  long long buffer;
  long long NewBuffer;
  int full;
} ledsBuf_t;

struct input_dev
{
  void* (*event) (void*);
  int led;
  pthread_mutex_t event_lock;
};

struct usb_kbd
{
  struct input_dev* dev;
  struct urb* irq;
  struct urb* led;
  
  unsigned char newleds, *leds;
  
  pthread_mutex_t leds_lock; 
  bool led_urb_submitted;
};

struct urb
{
  struct usb_kbd *context; 
  void* (*complete)(void* urb); 
  char transfer_buffer; 

  pthread_mutex_t transfer_buffer_lock; 
  thread_signal_t poll;
};


// usbkbd driver simulator
void usbkbd_driver_simulator();
int usb_kbd_open(struct input_dev* dev);
void usb_submit_urb(struct urb* urb);
void *usb_kbd_irq(void* urb);
void *usb_kbd_led(void* urb);
void *irq_endpoint();
void *cntl_endpoint();
void input_report_key(struct input_dev* dev, char keystroke);
void *usb_kbd_event(void* value);

// keyboard
void keyboard();
void *kb_endpointInterrupt();
void *kb_endpointControl();
void print_leds();

// syncronization used to wait for all simulation threads to set up before receiving data from the keyboard.
pthread_barrier_t waitForusbkbd;

// thread signals for both processes to signal shut down
thread_signal_t keyboardShutdown;
thread_signal_t driverShutdown;

// usbkbd data structures
struct input_dev dev;
struct usb_kbd kbd;
struct urb irq_urb;
struct urb led_urb;

pthread_t tid_usbkbd[5]; // simulator thread ids
pthread_t tid_kb[2]; // keyboard thread ids

// pipe file descriptors
int pipeInterrupt_fd[2];
int pipeControl_fd[2];
int pipeAck_fd[2];
int pipeSync[2];

// mmaped area (led buffer only ever has to hold 1 instruction)
ledsBuf_t* ledBuffer;

// This capslockEvents array stores all events during the simulation
long int capslockEvents[EVENT_SIZE];
int eventNo;

int main() 
{
  // set up the memory mapped area for led commands
  ledBuffer = (ledsBuf_t*)mmap(NULL, sizeof(ledsBuf_t), 
                                (PROT_READ | PROT_WRITE), (MAP_SHARED | MAP_ANON),
                                -1, 0);
  ledBuffer->buffer = 0;
  ledBuffer->full = 0;
  ledBuffer->NewBuffer = 0;

  if (pipe(pipeInterrupt_fd) == -1 || pipe(pipeControl_fd) == -1 || pipe(pipeAck_fd) == -1 || pipe(pipeSync) == -1)
      {
      printf("Error with Pipe\n");
	}
  switch(fork())
  {
    case -1:
      printf("Error with fork()\n");
      exit(0);

    case 0: // child
      usbkbd_driver_simulator();
      break;

    default: // parent
      keyboard();
      break;
  }
  printf("Error: terminated in main function\n");
	return 0;
}

// Process for usbkbd driver simulator
void usbkbd_driver_simulator() 
{
  // close unecessary ends of the pipes
  close(pipeInterrupt_fd[PIPE_WRITE]);
  close(pipeAck_fd[PIPE_WRITE]);       
  close(pipeControl_fd[PIPE_READ]);    
  close(pipeSync[PIPE_READ]);         

  // initialize shutdown signal
  driverShutdown.cond = 0;
  pthread_mutex_init(&driverShutdown.mutex, NULL);

  // define device parameters
  dev.event = &usb_kbd_event;
  dev.led = 0;
  pthread_mutex_init(&dev.event_lock, NULL);

  usb_kbd_open(&dev);

  // wait for the command to read from LED buffer 
  pthread_mutex_lock(&driverShutdown.mutex);
  while(driverShutdown.cond == 0)
  {
    pthread_mutex_unlock(&driverShutdown.mutex);
    sleep(0.1); // avoid starvation issues
    pthread_mutex_lock(&driverShutdown.mutex);
  }
  pthread_mutex_unlock(&driverShutdown.mutex);

  exit(0);
}

int usb_kbd_open(struct input_dev* dev)
{
  kbd.dev = dev;
  kbd.irq = &irq_urb;
  kbd.led = &led_urb;
  
  kbd.newleds = '0';
  kbd.leds = (unsigned char *)'0';
  kbd.led_urb_submitted = false;

  // configure urb parameters
  kbd.irq->complete = usb_kbd_irq;
  pthread_mutex_init(&kbd.irq->transfer_buffer_lock, NULL);

  kbd.led->complete = usb_kbd_led;
  pthread_mutex_init(&kbd.led->transfer_buffer_lock, NULL);

  usb_submit_urb(kbd.irq);
  return 0;
}


void usb_submit_urb(struct urb* urb) // usb_submit_urb
{
  int sync = 'S';
  static int count = 0; // defaults to 0

  // launch usb_kbd_irq and usb_kbd_led threads first time
  if (count == 0)
  {
    pthread_barrier_init (&waitForusbkbd, NULL, 3);
    pthread_create(&tid_usbkbd[0], NULL, irq_endpoint, (void*)kbd.irq);
    pthread_create(&tid_usbkbd[1], NULL, cntl_endpoint, (void*)kbd.led);
    pthread_detach(tid_usbkbd[0]);
    pthread_detach(tid_usbkbd[1]);

    // wait for simulation-side threads to get set up, then signal keyboard process
    pthread_barrier_wait (&waitForusbkbd);
    write(pipeSync[PIPE_WRITE], &sync, 1);
    count++;
  }

  pthread_mutex_lock(&urb->poll.mutex);
  urb->poll.cond = 1;
  pthread_mutex_unlock(&urb->poll.mutex);
}

void input_report_key(struct input_dev* dev, char keystroke)
{
  static long long capslockState = CAPSLOCK_OFF;

  // check if CAPSLOCK event needs to be started
  pthread_mutex_lock(&kbd.dev->event_lock);
  if (kbd.dev->led == 1)
  {
    pthread_mutex_unlock(&kbd.dev->event_lock);

    if (capslockState == CAPSLOCK_ON)
      capslockState = CAPSLOCK_OFF; 
    else
      capslockState = CAPSLOCK_ON; 

    // start the new keyboard event so the keyboard process can turn the led on
    pthread_create(&tid_usbkbd[4], NULL, kbd.dev->event, (void*)capslockState);
    pthread_detach(tid_usbkbd[4]);
    kbd.dev->led = 0;
    return;
  }
  pthread_mutex_unlock(&kbd.dev->event_lock);

  // Being handled in usb_kbd_irq, so ignoring here
  if (keystroke == '&')
    return;

  if (capslockState == CAPSLOCK_ON)
    putchar(toupper(keystroke));
  else
    putchar(keystroke);
}

void *usb_kbd_irq(void* surb){
  struct urb* urb = (struct urb*)surb;

  pthread_mutex_lock(&urb->transfer_buffer_lock);
  if (urb->transfer_buffer != '#')
  {

    pthread_mutex_lock(&kbd.dev->event_lock);
    if (urb->transfer_buffer == '@')
      kbd.dev->led = 1; // capslock event
    else
      kbd.dev->led = 0; // no capslock event
    pthread_mutex_unlock(&kbd.dev->event_lock);

    input_report_key(kbd.dev, urb->transfer_buffer);
  }
  pthread_mutex_unlock(&urb->transfer_buffer_lock);
  pthread_exit(NULL);
}

void *usb_kbd_led(void* surb) 
{
  struct urb* urb = (struct urb*)surb; 
  pthread_mutex_lock(&kbd.leds_lock); //Treating as spinlock
  
  if (ledBuffer->buffer == ledBuffer->NewBuffer){
  	kbd.led_urb_submitted = false;
  	pthread_mutex_unlock(&kbd.leds_lock);
  	pthread_exit(NULL);
  }
  
  ledBuffer->full = 1;
  ledBuffer->buffer = ledBuffer->NewBuffer;
  
  usb_submit_urb(kbd.led);
  
  pthread_mutex_unlock(&kbd.leds_lock);
  pthread_exit(NULL);
}


void *usb_kbd_event(void* value)
{
  
  pthread_mutex_lock(&kbd.leds_lock); //Treating as spinlock
  
  ledBuffer->NewBuffer = (long long)value;

  while (ledBuffer->full == 1)
  {
    pthread_mutex_unlock(&kbd.leds_lock);
    sleep(0.1);
    pthread_mutex_lock(&kbd.leds_lock);
  }
  
  if (kbd.led_urb_submitted){
  	pthread_mutex_unlock(&kbd.leds_lock);
  	pthread_exit(NULL);
  }
  
  if (ledBuffer->buffer == ledBuffer->NewBuffer){
  	pthread_mutex_unlock(&kbd.leds_lock);
  	pthread_exit(NULL);
  }
  
  ledBuffer->full = 1;
  ledBuffer->buffer = ledBuffer->NewBuffer;
  
  kbd.led_urb_submitted = true;
  usb_submit_urb(kbd.led);
  pthread_mutex_unlock(&kbd.leds_lock);
  
  pthread_exit(NULL);
}

//USB core
void *irq_endpoint(struct urb *urb)
{
  char keyboard_press;

  // sync all simulation side threads
  pthread_barrier_wait (&waitForusbkbd);

  while(1)
  {
    //Wait for polling
    pthread_mutex_lock(&urb->poll.mutex);
    while(urb->poll.cond == 0)
    {
      pthread_mutex_unlock(&urb->poll.mutex);
      sleep(0.1);
      pthread_mutex_lock(&urb->poll.mutex);
    }
    pthread_mutex_unlock(&urb->poll.mutex);

    if (read(pipeInterrupt_fd[PIPE_READ], &keyboard_press, 1) == 0){
      break;
    }

    //New usb_kbd_irq to handle keyboard press
    pthread_mutex_lock(&urb->transfer_buffer_lock);
    urb->transfer_buffer = keyboard_press;
    pthread_mutex_unlock(&urb->transfer_buffer_lock);
    
    pthread_create(&tid_usbkbd[2], NULL, urb->complete, (void*)urb);
    pthread_join(tid_usbkbd[2], NULL); //Wait for URB compeletion
  }

  pthread_mutex_lock(&driverShutdown.mutex);
  driverShutdown.cond = 1;
  pthread_mutex_unlock(&driverShutdown.mutex);
  pthread_exit(NULL);
}

void *cntl_endpoint(struct urb *urb)
{
  char cmd = 'C';
  char ack = 'X';

  // sync all usbkbd threads
  pthread_barrier_wait (&waitForusbkbd);

  while(1)
  {
    pthread_mutex_lock(&urb->poll.mutex);
    while(urb->poll.cond == 0)
    {
      pthread_mutex_unlock(&urb->poll.mutex);
      sleep(0.1);
      pthread_mutex_lock(&urb->poll.mutex);
    }

    urb->poll.cond = 0; 
    pthread_mutex_unlock(&urb->poll.mutex);

    write(pipeControl_fd[PIPE_WRITE], &cmd, 1);

    read(pipeAck_fd[PIPE_READ], &ack, 1);
    
    pthread_mutex_lock(&kbd.leds_lock);
    while(ack != 'A'){
	sleep(0.1);
    }
    
    kbd.led_urb_submitted = false;
    pthread_mutex_unlock(&kbd.leds_lock);
    
    pthread_create(&tid_usbkbd[3], NULL, usb_kbd_led, (void*)urb);
    pthread_join(tid_usbkbd[3], NULL); //Wait for URB compeletion
  }
  pthread_exit(NULL);
}

// Keyboard Process
void keyboard()
{
  eventNo = 0;

  // close unecessary ends of the pipes
  close(pipeInterrupt_fd[PIPE_READ]);
  close(pipeAck_fd[PIPE_READ]);      
  close(pipeControl_fd[PIPE_WRITE]);  
  close(pipeSync[PIPE_WRITE]);       

  // initializing shutdown signal
  keyboardShutdown.cond = 0;
  pthread_mutex_init(&keyboardShutdown.mutex, NULL);

  // Launch keyboard side endpoints
  pthread_create(&tid_kb[0], NULL, kb_endpointInterrupt, NULL);
  pthread_create(&tid_kb[1], NULL, kb_endpointControl, NULL);

  for (int i = 0; i < 2; i++){
    pthread_join(tid_kb[i], NULL);
    }
    
  print_leds();
  exit(0);
} 

void print_leds()
{
  for (int i = 0; i < eventNo; i++)
  { 
    if (capslockEvents[i] == 1)
      printf("ON ");
    else
      printf("OFF ");
  }
  printf("\n");
}

void *kb_endpointInterrupt() 
{
  char keyboardInput;
  char sync = 'X';

  // Waiting for usbkbd threads to initialize properly.
  while(sync != 'S')
  {
    read(pipeSync[PIPE_READ], &sync, 1);
  }

  while(read(STDIN_FILENO, &keyboardInput, 1) > 0)
  {
    write(pipeInterrupt_fd[PIPE_WRITE], &keyboardInput, 1); // redirect standard input to keyboard driver
  }

  sleep(1);
  pthread_mutex_lock(&keyboardShutdown.mutex);
  keyboardShutdown.cond = 1;
  pthread_mutex_unlock(&keyboardShutdown.mutex);
  
  
  pthread_exit(NULL);
}


void *kb_endpointControl()
{
  char ack = 'A';
  char cmd = 'X';
  int ledStatus = 0;
  int failCount = 0;

  fcntl(pipeControl_fd[PIPE_READ], F_SETFL, fcntl(pipeControl_fd[PIPE_READ], F_GETFL) | O_NONBLOCK);

  while(1)
  {
    pthread_mutex_lock(&keyboardShutdown.mutex);
    while(cmd != 'C' && keyboardShutdown.cond == 0)
    {
      pthread_mutex_unlock(&keyboardShutdown.mutex);
      read(pipeControl_fd[PIPE_READ], &cmd, 1);
      pthread_mutex_lock(&keyboardShutdown.mutex);
      sleep(0.1); // avoid starvation issues
    }
    pthread_mutex_unlock(&keyboardShutdown.mutex);

    pthread_mutex_lock(&kbd.leds_lock);
    
    while (ledBuffer->full == 0)
    {
      if (failCount++ == EVENT_SIZE)
        goto exit;
  
      pthread_mutex_unlock(&kbd.leds_lock);
      sleep(0.1);
      pthread_mutex_lock(&kbd.leds_lock);
    }

    failCount = 0;
    ledBuffer->full = 0; 
    ledStatus = ledBuffer->buffer;

    if (eventNo < EVENT_SIZE-1)
      capslockEvents[eventNo++] = ledStatus;

    pthread_mutex_unlock(&kbd.leds_lock);

    write(pipeAck_fd[PIPE_WRITE], &ack, 1);

    // Resetting cmd so as to make the thread wait for a control command from the usbkbd driver
    cmd = 'X';

    pthread_mutex_lock(&keyboardShutdown.mutex);
    if (keyboardShutdown.cond == 1)
    {
      pthread_mutex_unlock(&keyboardShutdown.mutex);
      break;
    }
    pthread_mutex_unlock(&keyboardShutdown.mutex);
  }
 
  exit:
  pthread_exit(NULL);
}

