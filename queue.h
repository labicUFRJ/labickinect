/*
 * Implements a queue for the type of RGBDImages with synchronized access control
 */

#ifndef QUEUE_H_
#define QUEUE_H_

#include <pthread.h>
#include <list>

namespace labic {
	template <class T>
	class Queue {
	public:
		Queue() {
			pthread_mutex_init(&mutex, NULL);
			pthread_cond_init(&condv, NULL);
		}

		~Queue() {
			pthread_mutex_destroy(&mutex);
			pthread_cond_destroy(&condv);
			//delete[] &queue;
		}

		void push(T item) {
			pthread_mutex_lock(&mutex);

			queue.push_back(item);
			//std::cout << "[Queue] New item added. Queue size: " << queue.size() << std::endl;

			pthread_cond_signal(&condv);
			pthread_mutex_unlock(&mutex);
		}

		T pop() {
			pthread_mutex_lock(&mutex);

			while (queue.size() == 0) {
				pthread_cond_wait(&condv, &mutex);
			}

			T item = queue.front();
			queue.pop_front();
			//std::cout << "[Queue] Item removed. Queue size: " << queue.size() << std::endl;

			pthread_mutex_unlock(&mutex);
			return item;
		}

		int size() {
			pthread_mutex_lock(&mutex);
			int size = queue.size();
			pthread_mutex_unlock(&mutex);
			return size;
		}

		void printStatus(std::ostream& o) const {
			o << "[Queue] Queue has " << queue.size() << " items";
		}

	private:
		std::list<T>	 queue;
		pthread_mutex_t	 mutex;
		pthread_cond_t	 condv;

	};

	template <class T>
	std::ostream& operator << (std::ostream& o, const Queue<T> queue) {
		queue.printStatus(o);
		return o;
	}

}


#endif /* QUEUE_H_ */
