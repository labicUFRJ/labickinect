/*
 * Implements a queue for the type of RGBDImages with synchronized access control
 */

#ifndef QUEUE_H_
#define QUEUE_H_

#include <pthread.h>
#include <list>
#include "RGBDImage.h"

namespace labic {
	class FrameQueue {
	public:
		FrameQueue() {
			pthread_mutex_init(&mutex, NULL);
			pthread_cond_init(&condv, NULL);
		}

		~FrameQueue() {
			pthread_mutex_destroy(&mutex);
			pthread_cond_destroy(&condv);
			delete[] &queue;
		}

		void push(RGBDImage image) {
			pthread_mutex_lock(&mutex);

			queue.push_back(image);
			//std::cout << "[FrameQueue] New frame added. Queue size: " << queue.size() << std::endl;

			pthread_cond_signal(&condv);
			pthread_mutex_unlock(&mutex);
		}

		RGBDImage pop() {
			pthread_mutex_lock(&mutex);

			while (queue.size() == 0) {
				pthread_cond_wait(&condv, &mutex);
			}

			RGBDImage image = queue.front();
			queue.pop_front();
			//std::cout << "[FrameQueue] Frame removed. Queue size: " << queue.size() << std::endl;

			pthread_mutex_unlock(&mutex);
			return image;
		}

		int size() {
			pthread_mutex_lock(&mutex);
			int size = queue.size();
			pthread_mutex_unlock(&mutex);
			return size;
		}

	private:
		std::list<RGBDImage> queue;
		pthread_mutex_t		 mutex;
		pthread_cond_t		 condv;

	};

}


#endif /* QUEUE_H_ */
