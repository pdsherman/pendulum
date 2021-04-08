/*
 * File:    main.cpp
 * Author:  pdsherman
 * Date:    April 2021
 */

#include <libs/threadsafe_queue/ThreadsafeQueue.hpp>
#include <libs/util/util.hpp>

#include <iostream>
#include <atomic>
#include <thread>
#include <random>
#include <chrono>
#include <string>
#include <vector>

 #include <unistd.h>

static ThreadsafeQueue<std::string> queue;
static std::atomic<bool> quit(false);
std::mutex write_mtx;

void queue_popping(int count, int id, int mint, int maxt)
{
  std::default_random_engine generator;
  std::uniform_int_distribution<int> distribution(mint, maxt);
  std::string tabs;
  for(int i = 0; i < id; ++i)
    tabs += "\t\t\t";

  int popped = 0;
  while(popped < count/2 && !quit.load()) {
    std::shared_ptr<std::string> ptr = queue.pop();
    if(ptr) {
      {
        std::lock_guard<std::mutex> lck(write_mtx);
        std::cout << tabs <<std::to_string(id) <<" Popped: " << *ptr << std::endl;
      }
      ++popped;
    }
    int number = distribution(generator);
    std::this_thread::sleep_for(std::chrono::milliseconds(number*100));
  }


}

int main(int argc, char* argv[])
{
  std::string filename = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/libs/threadsafe_queue/test/wordlist.txt";
  std::vector<std::string> lines = util::read_text_from_file(filename);

  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine generator(seed);
  std::uniform_int_distribution<int> distribution(0,10);

  // Begin popping thread1
  std::thread t1 = std::thread(queue_popping, lines.size(), 1, 5, 20);
  std::thread t2 = std::thread(queue_popping, lines.size(), 2, 19, 40);
  std::thread t3 = std::thread(queue_popping, lines.size(), 3, 3, 45);

  // Push words onto
  unsigned int pushed = 0;
  while(pushed < lines.size()) {
    queue.push(lines[pushed]);
    {
      std::lock_guard<std::mutex> lck(write_mtx);
      std::cout << "Pushed: " << lines[pushed] << std::endl;
    }
    ++pushed;
    int number = distribution(generator);
    std::this_thread::sleep_for(std::chrono::milliseconds(number*100));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(20000));

  // Ending
  quit = true;
  std::cout << "...Ending" << std::endl;
  if(t1.joinable())
    t1.join();
  if(t2.joinable())
    t2.join();
  if(t3.joinable())
    t3.join();
  return 0;
}
