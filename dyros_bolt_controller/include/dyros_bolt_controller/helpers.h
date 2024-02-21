#pragma once

#include <torch/script.h> // Include the PyTorch C++ API
#include <queue>

template <class T>
class limited_queue{
    public:

    limited_queue(int maxlen_) : maxlen(maxlen_){}

    void push(T input){
        if (my_queue.size() + 1 > maxlen){
            my_queue.pop();
        }
        my_queue.push(input);
    }

    T pop(){
        T ret = my_queue.front();
        my_queue.pop();
        return ret;
    }

    size_t size() const {return my_queue.size();}

    bool is_empty() const {return my_queue.empty();}

    std::queue<T> data() const {return my_queue;}

    private:
        size_t maxlen;
        std::queue<T> my_queue;
};


inline torch::Tensor queue_to_tensor(limited_queue<torch::Tensor> tensorQueue){
    // Check if the queue is empty
    if (tensorQueue.is_empty()) {
        // Return an empty tensor if the queue is empty
        throw std::invalid_argument("Queue cannot be empty");
    }

    // Get the first tensor from the queue

    torch::Tensor result = tensorQueue.pop(); // Remove the first tensor from the queue
    
    // Iterate over the remaining tensors in the queue
    while (!tensorQueue.is_empty()) {
        // Get the next tensor from the queue
        torch::Tensor nextTensor = tensorQueue.pop(); // Remove the tensor from the queue

        // Concatenate the next tensor along the specified dimension
        result = torch::cat({result, nextTensor}, 1);
        // result = torch::cat({nextTensor, result}, 1);
    }
    return result;
}

template <class T>
inline std::vector<T> queue_to_vector(const std::queue<T>& q) {
    std::vector<T> result;
    std::queue<T> tempQueue = q; // Create a copy of the input queue

    // Iterate through the elements of the queue and push them into the vector
    while (!tempQueue.empty()) {
        result.push_back(tempQueue.front());
        tempQueue.pop();
    }

    return result;
}