#ifndef WORLD_H_
#define WORLD_H_

#include "../common.h"
#include <thread>
#include <pcl/io/ply_io.h>

namespace labic {

	class World {
	public:
		static World& instance() {
			static World instance;
			return instance;
		}

		World& operator += (Cloud& c) {
			std::lock_guard updateLock(mutex);
			update = true;
			cloud += c;

			return *this;
		}

		bool getCloud(Cloud& output) {
			std::lock_guard updateLock(mutex);
			if (update) {
				output = cloud;
				return true;
			}
			return false;
		}

		Cloud::Ptr getCloudPtr() {
			return Cloud::Ptr(&cloud);
		}

		bool empty() { return cloud.empty(); }
		size_t size() { return cloud.size(); }

		void savePLY(std::string filename = "world.ply") {
			pcl::io::savePLYFileASCII(filename, cloud);
		}

	private:
		World() {}
		Cloud cloud;
		std::mutex mutex;
		bool update = false;
	};
}

#endif /* WORLD_H_ */
