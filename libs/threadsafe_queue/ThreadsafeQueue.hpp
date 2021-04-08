/*
 * File:    FunctionTimer.hpp
 * Author:  pdsherman
 * Date:    April 2021
 */

#pragma once

#include <memory>
#include <mutex>

template<typename T>
class ThreadsafeQueue
{
private:

/// Internal object to make up nodes in the queue
/// Contains value to data user wants stored
///  and pointer to next item in queue
struct Node
{
    std::shared_ptr<T> data;
    std::unique_ptr<Node> next;
};


public:

  /// Constructor
  ThreadsafeQueue(void) : _head(new Node()), _tail(_head.get()) {}

  /// Default Destrutor
  ~ThreadsafeQueue(void) = default;

  /// Get front item from queue
  /// @note Will return nullptr if queue is empty
  /// @return Pointer to data at front of queue (or NULL if empty)
  std::shared_ptr<T> pop(void);

  /// Push new item onto end of queue
  void push(T value);

  bool empty(void);

private:

  /// Get the raw point to tail.
  /// Used to compare against head for empty queue check
  Node *get_tail(void);

  /// Smart pointer to front of queue
  std::unique_ptr<Node> _head;

  /// Raw pointer to back of queue
  Node *_tail;

  /// For locking access to head node
  std::mutex _head_mtx;

  /// For locking access to tail node
  std::mutex _tail_mtx;

};

template<typename T>
std::shared_ptr<T> ThreadsafeQueue<T>::pop(void)
{
  std::shared_ptr<T> value;
  {
    std::lock_guard<std::mutex> lck(_head_mtx);
    if(_head.get() == get_tail())
      return nullptr;

    value = _head->data;
    _head = std::move(_head->next);
  }
  return value;
}

template<typename T>
void ThreadsafeQueue<T>::push(T value)
{
  std::unique_ptr<Node> new_tail(new Node());

  std::lock_guard<std::mutex> lck(_tail_mtx);
  _tail->data = std::make_shared<T>(value);
  _tail->next = std::move(new_tail);
  _tail       = _tail->next.get();
}

template<typename T>
bool ThreadsafeQueue<T>::empty(void)
{
  std::lock_guard<std::mutex> lck(_head_mtx);
  return _head.get() == get_tail();
}

template<typename T>
typename ThreadsafeQueue<T>::Node* ThreadsafeQueue<T>::get_tail(void)
{
  std::lock_guard<std::mutex> lck(_tail_mtx);
  return _tail;
}
