#include <iostream>
#include <uavcan_nuttx/uavcan_nuttx.hpp>
#include <uavcan/protocol/dynamic_node_id_client.hpp>
#include <uavcan/protocol/debug/KeyValue.hpp>
#include "debug.hpp"


//#define UAVCAN_HELLO_WORLD


uavcan_nuttx::NodePtr initNodeWithDynamicID(const std::vector<std::string>& ifaces,
		const std::uint8_t instance_id,
		const uavcan::NodeID preferred_node_id,
		const std::string& name)
{
	/*
	 * Initializing the node object
	 */
	auto node = uavcan_nuttx::makeNode(ifaces);

	node->setName(name.c_str());
	node->getLogger().setLevel(uavcan::protocol::debug::LogLevel::DEBUG);

	{
		const auto app_id = uavcan_nuttx::makeApplicationID(uavcan_nuttx::MachineIDReader().read(), name, instance_id);

		uavcan::protocol::HardwareVersion hwver;
		std::copy(app_id.begin(), app_id.end(), hwver.unique_id.begin());
		std::cout << hwver << std::endl;

		node->setHardwareVersion(hwver);
	}

	/*
	 * Starting the node
	 */
	const int start_res = node->start();

	std::cout << "start_res is " << start_res << std::endl;

	//ENFORCE(0 == start_res);

	/*
	 * Running the dynamic node ID client until it's done
	 */
	uavcan::DynamicNodeIDClient client(*node);

	const int client_start_res = client.start(node->getNodeStatusProvider().getHardwareVersion().unique_id, preferred_node_id);

	std::cout << "client start_res is " << client_start_res << std::endl;

	std::cout << "Waiting for dynamic node ID allocation..." << std::endl;

	while (!client.isAllocationComplete())
	{
		const int res = node->spin(uavcan::MonotonicDuration::fromMSec(100));
		if (res < 0)
		{
			std::cerr << "Spin error: " << res << std::endl;
		}
	}

	std::cout << "Node ID " << int(client.getAllocatedNodeID().get())
            								  << " allocated by " << int(client.getAllocatorNodeID().get()) << std::endl;

	/*
	 * Finishing the node initialization
	 */
	node->setNodeID(client.getAllocatedNodeID());

	node->setModeOperational();

	return node;
}

#define UAVCAN_HELLO_WORLD

void runForever(const uavcan_nuttx::NodePtr& node)
{
	int ret;

	/*uavcan::Subscriber<uavcan::protocol::debug::KeyValue> kv_sub(*node);

	const int kv_sub_start_res =
			kv_sub.start([&](const uavcan::protocol::debug::KeyValue& msg) {
		std::cout << msg << std::endl;
	});

	if (kv_sub_start_res < 0)
	{
		//throw std::runtime_error("Failed to start the key/value subscriber; error: " + std::to_string(kv_sub_start_res));
	}*/

	/*
	 * Running the node.
	 */
	//node.setModeOperational();

#ifdef UAVCAN_HELLO_WORLD
	uavcan::Publisher<uavcan::protocol::debug::KeyValue> kv_pub(*node);
	const int kv_pub_init_res = kv_pub.init();
	if (kv_pub_init_res < 0)
	{
		//throw std::runtime_error("Failed to start the publisher; error: " + std::to_string(kv_pub_init_res));
	}

	/*
	 * TX timeout can be overridden if needed.
	 * Default value should be OK for most use cases.
	 */
	kv_pub.setTxTimeout(uavcan::MonotonicDuration::fromMSec(1000));

	/*
	 * Priority of outgoing tranfers can be changed as follows.
	 * Default priority is 16 (medium).
	 */
	kv_pub.setPriority(uavcan::TransferPriority::MiddleLower);
#endif

	while (true)
	{
		const int res = node->spin(uavcan::MonotonicDuration::fromMSec(100));
		if (res < 0)
		{
			std::cerr << "Spin error: " << res << std::endl;
		}

#ifdef UAVCAN_HELLO_WORLD
		uavcan::protocol::debug::KeyValue kv_msg;  // Always zero initialized
		kv_msg.value = std::rand() / float(RAND_MAX);

		/*
		 * Arrays in DSDL types are quite extensive in the sense that they can be static,
		 * or dynamic (no heap needed - all memory is pre-allocated), or they can emulate std::string.
		 * The last one is called string-like arrays.
		 * ASCII strings can be directly assigned or appended to string-like arrays.
		 * For more info, please read the documentation for the class uavcan::Array<>.
		 */
		kv_msg.key = "H";   // "a"
		kv_msg.key += "e";  // "ab"
		kv_msg.key += "l";  // "abc"
		kv_msg.key += "l";  // "abc"
		kv_msg.key += "o";  // "abc"
		kv_msg.key += " ";  // "abc"
		kv_msg.key += "f";  // "abc"
		kv_msg.key += "r";  // "abc"
		kv_msg.key += "o";  // "abc"
		kv_msg.key += "m";  // "abc"
		kv_msg.key += " ";  // "abc"
		kv_msg.key += "n";  // "abc"
		kv_msg.key += "u";  // "abc"
		kv_msg.key += "t";  // "abc"
		kv_msg.key += "t";  // "abc"
		kv_msg.key += "x";  // "abc"

		/*
		 * Publishing the message.
		 */
		const int pub_res = kv_pub.broadcast(kv_msg);
		if (pub_res < 0)
		{
			std::cerr << "KV publication failure: " << pub_res << std::endl;
		}
#endif
	}
}


int cxxmain(int argc, char** argv)
{
    printf("CXX UAVCAN printf\r\n");
    std::vector<std::string> iface_names;
	const int self_node_id = 5;
	iface_names.push_back("can0"); //?
	//uavcan_nuttx::NodePtr node = initNode(iface_names, self_node_id, "org.uavcan.zephyr_test_node");
	uavcan_nuttx::NodePtr node = initNodeWithDynamicID(iface_names,
			std::uint8_t(self_node_id),
			std::uint8_t(self_node_id),
			"org.uavcan.nuttx_test_socketcan_client");
	std::cout << "Node initialized successfully" << std::endl;

    runForever(node);
	return 0;
}

extern "C"
{
  int main(int argc, char *argv[])
  {
    printf("Starting UAVCAN test node\r\n");
    return cxxmain(argc,argv);
  }
}
