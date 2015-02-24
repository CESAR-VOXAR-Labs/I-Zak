/**
* The Pillar Library
* Copyright (c) 2013 by Voxar Labs.-UFPE
* Permission to use, copy, modify, distribute and sell this software for any
*     purpose is hereby granted without fee, provided that the above copyright
*     notice appear in all copies and that both that copyright notice and this
*     permission notice appear in supporting documentation.
* The author makes no representations about the
*     suitability of this software for any purpose. It is provided "as is"
*     without express or implied warranty.
*/
#ifndef __Networking_H__
#define __Networking_H__

namespace LibTools
{
	/** \cond PRIVATE */
	enum Protocol
	{
		TCP,
		UDP
	};

	namespace Private
	{
		/**
		* Abstraction over specific api's socket implementation.<br>
		* Must be specialized.
		* @tparam protocol Protocol id flag.
		*/
		template<Protocol protocol>
		class SocketResolver
		{
		public:

			/**
			* Opens connection with some end point.
			* @param args connection attribute values.
			*/
			template<typename ... T>
			std::error_code open(T ... args);

			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			*/
			template<typename T>
			std::error_code send(T& dataToSend);

			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			* @param [out] lengthSent The actual data sent size.
			*/
			template<typename T>
			std::error_code send(T& dataToSend, unsigned int& lengthSent);

			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			* @param lengthToSend The data size.
			*/
			template<typename T>
			std::error_code send(T* dataToSend, unsigned int lengthToSend);

			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			* @param lengthToSend The data size.
			* @param [out] lengthSent The actual data sent size.
			*/
			template<typename T>
			std::error_code send(T* dataToSend, unsigned int lengthToSend, unsigned int& lengthSent);

			/**
			* Receives data from the connected end point.
			* @param dataToReceive Data that must be received.
			*/
			template<typename T>
			std::error_code receive(T& dataToReceive);

			/**
			* Receives data from the connected end point.
			* @param dataToReceive Data that must be received.
			* @param [out] lengthReceived The actual data received size.
			*/
			template<typename T>
			std::error_code receive(T& dataToReceive, unsigned int& lengthReceived);

			/**
			* Receives data from the connected end point.
			* @param dataToReceive Data that must be received.
			* @param lengthToReceive The data size.
			*/
			template<typename T>
			std::error_code receive(T* dataToReceive, unsigned int lengthToReceive);

			/**
			* Receives data from the connected end point.
			* @param dataToReceive Data that must be received.
			* @param lengthToReceive The data size.
			* @param [out] lengthReceived The actual data received size.
			*/
			template<typename T>
			std::error_code receive(T* dataToReceive, unsigned int lengthToReceive, unsigned int& lengthReceived);

			/**
			* Closes the connection with the connected end point.
			*/
			std::error_code close();
		};

		template<>
		class SocketResolver<TCP>
		{
		public:

			asio::io_service ioService;
			asio::ip::tcp::socket socket;

			/**
			* Default constructor.
			*/
			SocketResolver() : socket(ioService)
			{
			}
			
			/**
			* Opens a channel for remote end points connection.
			* @param v6 Is IP version 6?
			* @param port The port used for connection.
			* @param [out] address The remote end point address.
			* @return The socket error code.
			*/
			std::error_code open(bool v6, unsigned short port, std::string& address)
			{
				std::error_code error;

				asio::ip::tcp::acceptor acceptor(this->ioService, asio::ip::tcp::endpoint(v6 ? asio::ip::tcp::v6() : asio::ip::tcp::v4(), port));
				acceptor.accept(this->socket, error);
				address = this->socket.remote_endpoint().address().to_string();
				return error;
			}

			/**
			* Opens a connection with a remote end point.
			* @param address The remote end point address to connect.
			* @param port The remote end point port used for connection.
			* @param [out] v6 The remote end point is using IP version 6?
			* @return The socket error code.
			*/
			std::error_code open(const std::string& address, unsigned short port, bool& v6)
			{
				std::error_code error;
				std::ostringstream oss;
				oss << port;

				asio::ip::tcp::resolver resolver(this->ioService);
				asio::ip::tcp::resolver::query query(address, oss.str());
				asio::ip::tcp::resolver::iterator endPoint(resolver.resolve(query));

				v6 = endPoint->endpoint().address().is_v6();

				asio::connect(this->socket, endPoint, error);
				return error;
			}

			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code send(T& dataToSend)
			{
				std::error_code error;
				asio::write(this->socket, asio::buffer(dataToSend), error);
				return error;
			}

			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			* @param [out] lengthSent The actual data sent size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code send(T& dataToSend, unsigned int& lengthSent)
			{
				std::error_code error;
				lengthSent = asio::write(this->socket, asio::buffer(dataToSend), error);
				return error;
			}

			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			* @param lengthToSend The data size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code send(T* dataToSend, unsigned int lengthToSend = 1)
			{
				std::error_code error;
				asio::write(this->socket, asio::buffer(dataToSend, sizeof(T)*lengthToSend), error);
				return error;
			}
			
			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			* @param lengthToSend The data size.
			* @param [out] lengthSent The actual data sent size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code send(T* dataToSend, unsigned int lengthToSend, unsigned int& lengthSent)
			{
				std::error_code error;
				lengthSent = asio::write(this->socket, asio::buffer(dataToSend, sizeof(T)*lengthToSend), error);
				return error;
			}

			/**
			* Receives data from the connected end point.
			* @param [out] dataToReceive Data that must be received.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code receive(T& dataToReceive)
			{
				std::error_code error;
				this->socket.read_some(asio::buffer(dataToReceive), error);
				return error;
			}

			/**
			* Receives data from the connected end point.
			* @param [out] dataToReceive Data that must be received.
			* @param [out] lengthReceived The actual data received size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code receive(T& dataToReceive, unsigned int& lengthReceived)
			{
				std::error_code error;
				lengthReceived = this->socket.read_some(asio::buffer(dataToReceive), error);
				return error;
			}

			/**
			* Receives data from the connected end point.
			* @param [out] dataToReceive Data that must be received.
			* @param lengthToReceive The data size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code receive(T* dataToReceive, unsigned int lengthToReceive = 1)
			{
				std::error_code error;
				this->socket.read_some(asio::buffer(dataToReceive, sizeof(T)*lengthToReceive), error);
				return error;
			}

			/**
			* Receives data from the connected end point.
			* @param [out] dataToReceive Data that must be received.
			* @param lengthToReceive The data size.
			* @param [out] lengthReceived The actual data received size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code receive(T* dataToReceive, unsigned int lengthToReceive, unsigned int& lengthReceived)
			{
				std::error_code error;
				lengthReceived = this->socket.read_some(asio::buffer(dataToReceive, sizeof(T)*lengthToReceive), error);
				return error;
			}

			/**
			* Closes the connection with the connected end point.
			* @return The socket error code.
			*/
			std::error_code close()
			{
				std::error_code error;
				this->socket.shutdown(asio::socket_base::shutdown_both);
				this->socket.close(error);
				return error;
			}
		};

		template<>
		class SocketResolver<UDP>
		{
		public:

			asio::io_service ioService;
			asio::ip::udp::socket socket;

			/**
			* Default constructor.
			*/
			SocketResolver() : socket(ioService)
			{
			}

			/**
			* Opens a channel for remote end points connection.
			* @param v6 Is IP version 6?
			* @param port The port used for connection.
			* @param [out] address The remote end point address.
			* @return The socket error code.
			*/
			std::error_code open(bool v6, unsigned short port, std::string& address)
			{
				std::error_code error;
				char dummy;

				this->socket = asio::ip::udp::socket(this->ioService, asio::ip::udp::endpoint(v6 ? asio::ip::udp::v6() : asio::ip::udp::v4(), port));
				asio::ip::udp::endpoint remote_endpoint;
				this->socket.receive_from(asio::buffer(&dummy, sizeof(char)), remote_endpoint, 0, error);
				this->socket.connect(remote_endpoint);
				address = remote_endpoint.address().to_string();

				return error;
			}

			/**
			* Opens a connection with a remote end point.
			* @param address The remote end point address to connect.
			* @param port The remote end point port used for connection.
			* @param [out] v6 The remote end point is using IP version 6?
			* @return The socket error code.
			*/
			std::error_code open(const std::string& address, unsigned short port, bool& v6)
			{
				std::error_code error;
				char dummy;

				std::ostringstream oss;
				oss << port;

				asio::ip::udp::resolver resolver(this->ioService);
				asio::ip::udp::resolver::query query(address, oss.str());
				asio::ip::udp::endpoint receiver_endpoint = *resolver.resolve(query);
				v6 = receiver_endpoint.address().is_v6();

				this->socket.connect(receiver_endpoint, error);
				this->socket.send(asio::buffer(&dummy, sizeof(char)));
				return error;
			}

			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code send(T& dataToSend)
			{
				std::error_code error;
				this->socket.send(asio::buffer(dataToSend), 0, error);
				return error;
			}

			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			* @param [out] lengthSent The actual data sent size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code send(T& dataToSend, unsigned int& lengthSent)
			{
				std::error_code error;
				lengthSent = this->socket.send(asio::buffer(dataToSend), 0, error);
				return error;
			}

			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			* @param lengthToSend The data size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code send(T* dataToSend, unsigned int lengthToSend = 1)
			{
				std::error_code error;
				this->socket.send(asio::buffer(dataToSend, sizeof(T)*lengthToSend), 0, error);
				return error;
			}

			/**
			* Sends data to the connected end point.
			* @param dataToSend Data that must be sent.
			* @param lengthToSend The data size.
			* @param [out] lengthSent The actual data sent size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code send(T* dataToSend, unsigned int lengthToSend, unsigned int& lengthSent)
			{
				std::error_code error;
				lengthSent = this->socket.send(asio::buffer(dataToSend, sizeof(T)*lengthToSend), 0, error);
				return error;
			}

			/**
			* Receives data from the connected end point.
			* @param [out] dataToReceive Data that must be received.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code receive(T& dataToReceive)
			{
				std::error_code error;
				this->socket.receive(asio::buffer(dataToReceive), 0, error);
				return error;
			}

			/**
			* Receives data from the connected end point.
			* @param [out] dataToReceive Data that must be received.
			* @param [out] lengthReceived The actual data received size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code receive(T& dataToReceive, unsigned int& lengthReceived)
			{
				std::error_code error;
				lengthReceived = this->socket.receive(asio::buffer(dataToReceive), 0, error);
				return error;
			}

			/**
			* Receives data from the connected end point.
			* @param [out] dataToReceive Data that must be received.
			* @param lengthToReceive The data size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code receive(T* dataToReceive, unsigned int lengthToReceive = 1)
			{
				std::error_code error;
				lengthReceived = this->socket.receive(asio::buffer(dataToReceive, sizeof(T)*lengthToReceive), 0, error);
				return error;
			}

			/**
			* Receives data from the connected end point.
			* @param [out] dataToReceive Data that must be received.
			* @param lengthToReceive The data size.
			* @param [out] lengthReceived The actual data received size.
			* @return The socket error code.
			*/
			template<typename T>
			std::error_code receive(T* dataToReceive, unsigned int lengthToReceive, unsigned int& lengthReceived)
			{
				std::error_code error;
				lengthReceived = this->socket.receive(asio::buffer(dataToReceive, sizeof(T)*lengthToReceive), 0, error);
				return error;
			}

			/**
			* Closes the connection with the connected end point.
			* @return The socket error code.
			*/
			std::error_code close()
			{
				std::error_code error;
				this->socket.shutdown(asio::socket_base::shutdown_both);
				this->socket.close(error);
				return error;
			}
		};
	}
	/** \endcond */

	/**
	* Network node (over IP) interface.<br>
	* Must not be used as concrete object.
	*/
	template<Protocol protocol>
	class NetNode
	{	
	protected:

		/** Connection port number. */
		unsigned short port;
		/** Remote end point address.*/
		std::string address;
		/** IP version 6 flag.*/
		bool ipV6;

		/** Socke implementation used for the connection.*/
		Private::SocketResolver<protocol> resolver;

	public:

		/**
		* Constructor used for client-like nodes.
		* @param address Remote end point address.
		* @param port Remote end point port.
		*/
		NetNode(const std::string& address, const unsigned short port) :
			address(address),
			port(port)
		{}

		/**
		* Constructor used for server-like nodes.		
		* @param port The port to be openned.
		* @param ipV6 Is this connection using IP version 6?
		*/
		NetNode(const unsigned short port, bool ipV6) :
			port(port),
			ipV6(ipV6)
		{}

		/**
		* Sends data to the connected end point.
		* @param dataToSend Data that must be sent.
		* @return The socket error code.
		*/
		template<typename T>
		std::error_code send(T* dataToSend, unsigned int lengthToSend = 1)
		{
			return this->resolver.send(dataToSend, lengthToSend);
		}

		/**
		* Sends data to the connected end point.
		* @param dataToSend Data that must be sent.
		* @param [out] lengthSent The actual data sent size.
		* @return The socket error code.
		*/
		template<typename T>
		std::error_code send(T* dataToSend, unsigned int lengthToSend, unsigned int& lengthSent)
		{
			return this->resolver.send(dataToSend, lengthToSend, lengthSent);
		}

		/**
		* Sends data to the connected end point.
		* @param dataToSend Data that must be sent.
		* @param lengthToSend The data size.
		* @return The socket error code.
		*/
		template<typename T>
		std::error_code send(T& dataToSend)
		{
			return this->resolver.send(dataToSend);
		}

		/**
		* Sends data to the connected end point.
		* @param dataToSend Data that must be sent.
		* @param lengthToSend The data size.
		* @param [out] lengthSent The actual data sent size.
		* @return The socket error code.
		*/
		template<typename T>
		std::error_code send(T& dataToSend, unsigned int& lengthSent)
		{
			return this->resolver.send(dataToSend, lengthSent);
		}

		/**
		* Receives data from the connected end point.
		* @param [out] dataToReceive Data that must be received.
		* @return The socket error code.
		*/
		template<typename T>
		std::error_code receive(T& dataToReceive)
		{
			return this->resolver.receive(dataToReceive);
		}

		/**
		* Receives data from the connected end point.
		* @param [out] dataToReceive Data that must be received.
		* @param [out] lengthReceived The actual data received size.
		* @return The socket error code.
		*/
		template<typename T>
		std::error_code receive(T& dataToReceive, unsigned int& lengthReceived)
		{
			return this->resolver.receive(dataToReceive, lengthReceived);
		}

		/**
		* Receives data from the connected end point.
		* @param [out] dataToReceive Data that must be received.
		* @param lengthToReceive The data size.
		* @return The socket error code.
		*/
		template<typename T>
		std::error_code receive(T* dataToReceive, unsigned int lengthToReceive = 1)
		{
			return this->resolver.receive(dataToReceive, lengthToReceive);
		}

		/**
		* Receives data from the connected end point.
		* @param [out] dataToReceive Data that must be received.
		* @param lengthToReceive The data size.
		* @param [out] lengthReceived The actual data received size.
		* @return The socket error code.
		*/
		template<typename T>
		std::error_code receive(T* dataToReceive, unsigned int lengthToReceive, unsigned int& lengthReceived)
		{
			return this->resolver.receive(dataToReceive, lengthToReceive, lengthReceived);
		}

		/**
		* Closes the connection with the connected end point.
		* @return The socket error code.
		*/
		std::error_code close()
		{
			return this->resolver.close();
		}

		/**
		* Returns the used port number.
		* @return The port number.
		*/
		unsigned int getPort() const
		{
			return this->port;
		}

		/**
		* Returns the remote address.
		* @return The remote address.
		*/
		const std::string& getAddress() const
		{
			return this->address;
		}

		/**
		* Is the connection over IP version 6?
		* @return true if it is.
		*/
		bool isIpV6() const
		{
			return this->ipV6;
		}
	};	

	/**
	* TCP server implementation.
	*/
	class Server : public NetNode<TCP>
	{
	public:

		/**
		* Constructor.
		* @param port Port number used to receive connections.
		* @param tcp6 Is the connection using IP version 6.
		*/
		Server(const unsigned short port, bool tcp6 = false) :
			NetNode(port, tcp6)
		{
		}

		/**
		* Accepts incoming connections.<br>
		* Only one openned connection is alowed per time.
		* @return The socket error code.
		*/
		std::error_code accept()
		{
			return this->resolver.open(this->ipV6, this->port, this->address);
		}
	};

	/**
	* TCP client implementation.
	*/
	class Client : public NetNode<TCP>
	{
	public:

		/**
		* Constructor.		
		* @param address Remote server address.
		* @param port Remote server port.
		*/
		Client(const std::string& address, const unsigned short port) :
			NetNode(address, port)
		{
		}

		/**
		* Connects to the server.
		* Only one openned connection is alowed per time.
		* @return The socket error code.
		*/
		std::error_code connect()
		{
			return this->resolver.open(this->address, this->port, this->ipV6);
		}
	};

	class Peer : public NetNode<UDP>
	{
	public:

		Peer(const unsigned short port, bool udp6 = false) :
			NetNode(port, udp6)
		{
		}

		Peer(const std::string& address, const unsigned short port) :
			NetNode(address, port)
		{
		}

		std::error_code open()
		{
			return address.length() > 0 ?
				this->resolver.open(this->address, this->port, this->ipV6)
				:
				this->resolver.open(this->ipV6, this->port, this->address);
		}
	};
}

#endif