#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fcntl.h>
#include <pthread.h>
#include <linux/input.h>

#define KEYBOARD_DEV "/dev/input/event5"   //5?

struct keyboard_state {
    signed short keys[KEY_CNT];
};

class cKeyboard {
  private:

    bool active;
    int keyboard_fd;
    input_event *keyboard_ev;
    keyboard_state *keyboard_st;
    char name[256];

  protected:
  public:
    cKeyboard();
    ~cKeyboard();
    static void* loop(void* obj);
    void readEv();
    short getKeyState(short key);
    pthread_t thread;
};

#endif