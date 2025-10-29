// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class EricTest {
  private static AtomicBoolean running = new AtomicBoolean(true);
  private static AtomicBoolean strobe = new AtomicBoolean(false);

  private static AtomicInteger motorSpeed1 = new AtomicInteger(0);
  private static AtomicInteger motorSpeed2 = new AtomicInteger(0);
  private static AtomicInteger motorSpeed3 = new AtomicInteger(0);
  private static AtomicInteger motorSpeed4 = new AtomicInteger(0);

  private static void sleep(int ms) {
    try {
      Thread.sleep(ms);
    } catch (InterruptedException ex) {

    }
  }

  public static void main(String[] args) {
    ArrayList<Thread> motorThreads = new ArrayList<Thread>();

    motorThreads.add(
        new Thread() {
          public void run() {
            boolean printOnce = false;
            System.out.println("Motor 1 initialized");
            while (running.get()) {
              if (strobe.get()) {
                if (!printOnce) {
                  printOnce = true;
                  int speed = motorSpeed1.get();
                  System.out.println("Motor 1 speed: " + speed);
                }
              } else {
                printOnce = false;
              }
            }
          }
        });

    motorThreads.add(
        new Thread() {
          public void run() {
            boolean printOnce = false;
            System.out.println("Motor 2 initialized");
            while (running.get()) {
              if (strobe.get()) {
                if (!printOnce) {
                  printOnce = true;
                  int speed = motorSpeed2.get();
                  System.out.println("Motor 2 speed: " + speed);
                }
              } else {
                printOnce = false;
              }
            }
          }
        });

    motorThreads.add(
        new Thread() {
          public void run() {
            boolean printOnce = false;
            System.out.println("Motor 3 initialized");
            while (running.get()) {
              if (strobe.get()) {
                if (!printOnce) {
                  printOnce = true;
                  int speed = motorSpeed3.get();
                  System.out.println("Motor 3 speed: " + speed);
                }
              } else {
                printOnce = false;
              }
            }
          }
        });

    motorThreads.add(
        new Thread() {
          public void run() {
            boolean printOnce = false;
            System.out.println("Motor 4 initialized");
            while (running.get()) {
              if (strobe.get()) {
                if (!printOnce) {
                  printOnce = true;
                  int speed = motorSpeed4.get();
                  System.out.println("Motor 4 speed: " + speed);
                }
              } else {
                printOnce = false;
              }
            }
          }
        });

    // Start all motor threads
    for (Thread t : motorThreads) {
      t.start();
    }

    System.out.println("Loading motor speeds");

    motorSpeed1.set(4);
    motorSpeed2.set(5);
    motorSpeed3.set(6);
    motorSpeed4.set(7);

    sleep(5000);

    strobe.set(true);
    sleep(10);
    strobe.set(false);
  }
}
