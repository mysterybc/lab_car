#include "msgparser.hpp"
#include <cstdio>
#include <vector>

struct BinarProtocol {
	static std::uint32_t header;
	static std::uint8_t  tail;
};
std::uint32_t BinarProtocol::header = 0xafbec1d9;
std::uint8_t BinarProtocol::tail = 0;


// For the current purpose, 
// The binary format on both machines are the same
// So we will just send the data as raw bytes

namespace ssbinary {
	template<class T>
	std::size_t size_require(const T* data, std::size_t length) {
		return sizeof(std::uint16_t) + sizeof(T)*length;
	}
	
    template<class T>
	bool pack(WriteBuffer& buffer, const T& value) {
		std::size_t size = sizeof(value);
		if (buffer.size >= size) {
			T* p_data = (T*)buffer.buffer;
			*p_data = value;
			buffer.eat(size);
			return true;
		}
		return false;
	}
	template<>
	bool pack(WriteBuffer& buffer, const RobotConfigMap::Point& data) {
		return pack(buffer, data.x)
			&& pack(buffer, data.y);
	}
	template<>
	bool pack(WriteBuffer& buffer, const RobotConfigMap::Circle& data) {
		return pack(buffer, data.q)
			&& pack(buffer, data.r);
	}
	template<>
	bool pack(WriteBuffer& buffer, const RobotConfigMap::Line& data) {
		return pack(buffer, data.p)
			&& pack(buffer, data.q);
	}
	template<class T>
	bool pack(WriteBuffer& buffer, const T* data, std::size_t length) {
		std::size_t size_need = size_require(data, length);
		if (length >= 0xffff) {
			printf("Error at BinarySerializer pack_array(), array size too large\n");
			return false;
		}
		if (buffer.size >= size_need) {
			pack(buffer, (std::uint16_t)length);
			T* value = (T*)data;
			for (std::size_t i=0;i<length;++i) pack(buffer, value[i]);
			return true;
		}
		return false;
	}
	

}


struct BinarySerializer: public SerializerBase {
	/*
	template<class T>
	bool pack(WriteBuffer& buffer, const T& value) {
		std::size_t size = sizeof(value);
		if (buffer.size >= size) {
			T* p_data = (T*)buffer.buffer;
			*p_data = value;
			buffer.eat(size);
			return true;
		}
		return false;
	}
	template<>
	bool pack(WriteBuffer& buffer, const RobotConfigMap::Point& data) {
		return pack(buffer, data.x)
			&& pack(buffer, data.y);
	}
	template<>
	bool pack(WriteBuffer& buffer, const RobotConfigMap::Circle& data) {
		return pack(buffer, data.q)
			&& pack(buffer, data.r);
	}
	template<>
	bool pack(WriteBuffer& buffer, const RobotConfigMap::Line& data) {
		return pack(buffer, data.p)
			&& pack(buffer, data.q);
	}
	template<class T>
	bool pack(WriteBuffer& buffer, const T* data, std::size_t length) {
		std::size_t size_need = size_require(data, length);
		if (length >= 0xffff) {
			printf("Error at BinarySerializer pack_array(), array size too large\n");
			return false;
		}
		if (buffer.size >= size_need) {
			pack(buffer, (std::uint16_t)length);
			T* value = (T*)data;
			for (std::size_t i=0;i<length;++i) pack(buffer, value[i]);
			return true;
		}
		return false;
	}
	template<class T>
	std::size_t size_require(const T* data, std::size_t length) {
		return sizeof(std::uint16_t) + sizeof(T)*length;
	}
	*/
	std::size_t size_header() {
		using namespace ssbinary;
		static PackageInfo info;
		static std::size_t size = 0;
		if (size == 0) {
			size += sizeof(BinarProtocol::header);
			size += sizeof(info.data_size);
			size += sizeof(info.package_type);
			size += sizeof(info.senderID);
			size += sizeof(info.targetID);
		}
		return size;
	}
	bool write_header(WriteBuffer buffer, const PackageInfo& info) {
		using namespace ssbinary;
		std::size_t size = size_header();
		if (buffer.size >= size) {
			pack(buffer, BinarProtocol::header);
			pack(buffer, info.data_size);
			pack(buffer, info.package_type);
			pack(buffer, info.senderID);
			pack(buffer, info.targetID);
			return true;
		}
		return false;
	}
	
	int write(WriteBuffer buffer, PackageInfo& info, const RobotStateLog& data) {
		using namespace ssbinary;
		WriteBuffer bf = buffer;
		if (!bf.eat(size_header()))  return 0;
		if (!pack(bf, data.loop_counter))        return 0;
		if (!pack(bf, data.task_motor)) 	     return 0;
		if (!pack(bf, data.task_aux)) 	         return 0;
		if (!pack(bf, data.x))                   return 0;
		if (!pack(bf, data.y)) 	                 return 0;
		if (!pack(bf, data.heading))             return 0;
		if (!pack(bf, data.uv))                  return 0;
		if (!pack(bf, data.uw))                  return 0;
		if (!pack(bf, data.xd))                  return 0;
		if (!pack(bf, data.yd))                  return 0;
		if (!pack(bf, data.task_progess))        return 0;
		
		info.data_size = (std::uint32_t)(buffer.size - bf.size);
		info.package_type = PackageInfo::PackRobotStateLog;
		write_header(buffer, info);
		return (int)info.data_size;
	}
	
	int write(WriteBuffer buffer, PackageInfo& info, const RobotCommand& data) {
		using namespace ssbinary;
		WriteBuffer bf = buffer;
		if (!bf.eat(size_header()))			     return 0;
		if (!pack(bf, data.command))             return 0;
		if (!pack(bf, data.command_ex)) 	     return 0;
		if (!pack(bf, data.dataX)) 	             return 0;
		if (!pack(bf, data.dataY))               return 0;
		
		info.data_size = (std::uint32_t)(buffer.size - bf.size);
		info.package_type = PackageInfo::PackRobotCommand;
		write_header(buffer, info);
		return (int)info.data_size;
	}
	int write(WriteBuffer buffer, PackageInfo& info, const RobotTeleMsg& data) {
		using namespace ssbinary;
		WriteBuffer bf = buffer;
		if (!bf.eat(size_header()))	return 0;
		if (!pack(bf, data.v))     return 0;
		if (!pack(bf, data.w)) 	return 0;
		
		info.data_size = (std::uint32_t)(buffer.size - bf.size);
		info.package_type = PackageInfo::PackRobotTeleMsg;
		write_header(buffer, info);
		return (int)info.data_size;
	}
	int write(WriteBuffer buffer, PackageInfo& info, const RobotConfigState& data) {
		using namespace ssbinary;
		WriteBuffer bf = buffer;
		if (!bf.eat(size_header()))	 return 0;
		if (!pack(bf, data.x))       return 0;
		if (!pack(bf, data.y)) 	     return 0;
		if (!pack(bf, data.vx))      return 0;
		if (!pack(bf, data.vy)) 	 return 0;
		if (!pack(bf, data.heading)) return 0;
		
		info.data_size = (std::uint32_t)(buffer.size - bf.size);
		info.package_type = PackageInfo::PackRobotConfigState;
		write_header(buffer, info);
		return (int)info.data_size;
	}
	int write(WriteBuffer buffer, PackageInfo& info, const RobotConfigPath& data) {
		using namespace ssbinary;
		WriteBuffer bf = buffer;
		if (!bf.eat(size_header()))  return 0;
		if (!pack(bf, data.pathType))            return 0;
		if (!pack(bf, data.uniform_velocity))    return 0;
		if (!pack(bf, &data.qx.front(), data.qx.size())) return 0;
		if (!pack(bf, &data.qy.front(), data.qy.size())) return 0;
		if (!pack(bf, &data.vd.front(), data.vd.size())) return 0;
		if (!pack(bf, &data.wd.front(), data.wd.size())) return 0;
		
		info.data_size = (std::uint32_t)(buffer.size - bf.size);
		info.package_type = PackageInfo::PackRobotConfigPath;
		write_header(buffer, info);
		return (int)info.data_size;
	}
	int write(WriteBuffer buffer, PackageInfo& info, const RobotConfigMap& data) {
		using namespace ssbinary;
		WriteBuffer bf = buffer;
		if (!bf.eat(size_header()))  return 0;
		if (!pack(bf, data.command)) return 0;
		if (!pack(bf, &data.obPoint.front(), data.obPoint.size())) return 0;
		if (!pack(bf, &data.obLine.front(), data.obLine.size())) return 0;
		if (!pack(bf, &data.obCPoly.front(), data.obCPoly.size())) return 0;
		
		
		info.data_size = (std::uint32_t)(buffer.size - bf.size);
		info.package_type = PackageInfo::PackRobotConfigMap;
		write_header(buffer, info);
		return (int)info.data_size;
	}
};

namespace ssbinary {
	// Utility Function
	template<class T>
	bool unpack(ReadBuffer& buffer, T& data) {
		if (buffer.size >= sizeof(data)) {
			data = *((const T*)buffer.buffer);
			buffer.eat(sizeof(data));
			return true;
		}
		return false;
	}
	template<>
	bool unpack(ReadBuffer& buffer, RobotConfigMap::Point& data) {
		return unpack(buffer, data.x)
		    && unpack(buffer, data.y);
	}
	template<>
	bool unpack(ReadBuffer& buffer, RobotConfigMap::Circle& data) {
		return unpack(buffer, data.q)
			&& unpack(buffer, data.r);
	}
	template<>
	bool unpack(ReadBuffer& buffer, RobotConfigMap::Line& data) {
		return unpack(buffer, data.p)
			&& unpack(buffer, data.q);
	}

	template<class T>
	bool unpack(ReadBuffer& buffer, std::vector<T>& data) {
		std::uint16_t length;
		bool good = true;
		good = good && unpack(buffer, length);
		data.resize(length);
		for (std::uint16_t i=0; i<length; ++i) {
			good = good && unpack(buffer, data[i]);
		}
		return good;
	}
}

struct BinaryParser: public ParserBase {
	/*
	// Utility Function
	template<class T>
	bool unpack(ReadBuffer& buffer, T& data) {
		if (buffer.size >= sizeof(data)) {
			data = *((const T*)buffer.buffer);
			buffer.eat(sizeof(data));
			return true;
		}
		return false;
	}
	template<>
	bool unpack(ReadBuffer& buffer, RobotConfigMap::Point& data) {
		return unpack(buffer, data.x)
		    && unpack(buffer, data.y);
	}
	template<>
	bool unpack(ReadBuffer& buffer, RobotConfigMap::Circle& data) {
		return unpack(buffer, data.q)
			&& unpack(buffer, data.r);
	}
	template<>
	bool unpack(ReadBuffer& buffer, RobotConfigMap::Line& data) {
		return unpack(buffer, data.p)
			&& unpack(buffer, data.q);
	}

	template<class T>
	bool unpack(ReadBuffer& buffer, std::vector<T>& data) {
		std::uint16_t length;
		bool good = true;
		good = good && unpack(buffer, length);
		data.resize(length);
		for (std::uint16_t i=0; i<length; ++i) {
			good = good && unpack(buffer, data[i]);
		}
		return good;
	}
	*/
	bool read_header(ReadBuffer& buffer, PackageInfo& info) {
		using namespace ssbinary;
		std::uint32_t header = 0;
		if (!unpack(buffer, header))			return false;
		if (header != BinarProtocol::header)    return false;
		if (!unpack(buffer, info.data_size))    return false;
		if (!unpack(buffer, info.package_type)) return false;
		if (!unpack(buffer, info.senderID))     return false;
		if (!unpack(buffer, info.targetID))     return false;
		return true;
	}
	std::size_t size_header() {
		static BinarySerializer ser;
		return ser.size_header();
	}
	
	// Interface Function : Used Split the data
	int find_packet(ReadBuffer buffer, PackageInfo& info) {
		const char* buf_end = buffer.buffer + buffer.size;
		const char* ptr_end = buffer.buffer + buffer.size - size_header();
		for (const char* ptr = buffer.buffer; ptr < ptr_end; ++ptr) {
			ReadBuffer tmp(ptr, buf_end);
			if (read_header(tmp, info)) {
				return ptr - buffer.buffer;
			}
		}
		return -1;
	}
	
	// Interface Function
	int read(ReadBuffer buffer, PackageInfo& info, RobotStateLog& data) {
		using namespace ssbinary;
		ReadBuffer bf = buffer;
		if (!read_header(bf, info))                 return 0;
		if (info.package_type != PackageInfo::PackRobotStateLog) return 0;
		if (buffer.size < info.data_size)           return 0;
		
		if (!unpack(bf, data.loop_counter)) return 0;
		if (!unpack(bf, data.task_motor))   return 0;
		if (!unpack(bf, data.task_aux))     return 0;
		if (!unpack(bf, data.x))            return 0;
		if (!unpack(bf, data.y))            return 0;
		if (!unpack(bf, data.heading))      return 0;
		if (!unpack(bf, data.uv))           return 0;
		if (!unpack(bf, data.uw))           return 0;
		if (!unpack(bf, data.xd))           return 0;
		if (!unpack(bf, data.yd))           return 0;
		if (!unpack(bf, data.task_progess)) return 0;
					
		// Check that each member has been unpacked
		// and the bytes number mathes what has been specified in data_size
		std::size_t size_read = buffer.size - bf.size;
		return size_read == info.data_size ? (int)info.data_size : 0;
	}
	int read(ReadBuffer buffer, PackageInfo& info, RobotCommand& data) {
		using namespace ssbinary;
		ReadBuffer bf = buffer;
		if (!read_header(bf, info))                 return 0;
		if (info.package_type != PackageInfo::PackRobotCommand)  return 0;
		if (buffer.size < info.data_size)           return 0;
		
		if (!unpack(bf, data.command))    return 0;
		if (!unpack(bf, data.command_ex)) return 0;
		if (!unpack(bf, data.dataX))      return 0;
		if (!unpack(bf, data.dataY))      return 0;
					
		// Check that each member has been unpacked
		// and the bytes number mathes what has been specified in data_size
		std::size_t size_read = buffer.size - bf.size;
		return size_read == info.data_size ? (int)info.data_size : 0;
	}
	int read(ReadBuffer buffer, PackageInfo& info, RobotTeleMsg& data) {
		using namespace ssbinary;
		ReadBuffer bf = buffer;
		if (!read_header(bf, info))                 return 0;
		if (info.package_type != PackageInfo::PackRobotTeleMsg)  return 0;
		if (buffer.size < info.data_size)           return 0;
		
		if (!unpack(bf, data.v))    return 0;
		if (!unpack(bf, data.w))    return 0;
					
		// Check that each member has been unpacked
		// and the bytes number mathes what has been specified in data_size
		std::size_t size_read = buffer.size - bf.size;
		return size_read == info.data_size ? (int)info.data_size : 0;
	} 
	int read(ReadBuffer buffer, PackageInfo& info, RobotConfigState& data) {
		using namespace ssbinary;
		ReadBuffer bf = buffer;
		if (!read_header(bf, info))                 return 0;
		if (info.package_type != PackageInfo::PackRobotConfigState)  return 0;
		if (buffer.size < info.data_size)           return 0;
		
		if (!unpack(bf, data.x))    return 0;
		if (!unpack(bf, data.y))    return 0;
		if (!unpack(bf, data.vx))   return 0;
		if (!unpack(bf, data.vy))   return 0;
		if (!unpack(bf, data.heading))    return 0;
					
		// Check that each member has been unpacked
		// and the bytes number mathes what has been specified in data_size
		std::size_t size_read = buffer.size - bf.size;
		return size_read == info.data_size ? (int)info.data_size : 0;
	}
	int read(ReadBuffer buffer, PackageInfo& info, RobotConfigPath& data) {
		using namespace ssbinary;
		ReadBuffer bf = buffer;
		if (!read_header(bf, info))                 return 0;
		if (info.package_type != PackageInfo::PackRobotConfigPath)  return 0;
		if (buffer.size < info.data_size)           return 0;
		
		if (!unpack(bf, data.pathType))    return 0;
		if (!unpack(bf, data.uniform_velocity))    return 0;
		if (!unpack(bf, data.qx))   return 0;
		if (!unpack(bf, data.qy))   return 0;
		if (!unpack(bf, data.vd))   return 0;
		if (!unpack(bf, data.wd))   return 0;
					
		// Check that each member has been unpacked
		// and the bytes number mathes what has been specified in data_size
		std::size_t size_read = buffer.size - bf.size;
		return size_read == info.data_size ? (int)info.data_size : 0;
	} 
	int read(ReadBuffer buffer, PackageInfo& info, RobotConfigMap& data) {
		using namespace ssbinary;
		ReadBuffer bf = buffer;
		if (!read_header(bf, info))                 return 0;
		if (info.package_type != PackageInfo::PackRobotConfigMap)  return 0;
		if (buffer.size < info.data_size)           return 0;
		
		if (!unpack(bf, data.command)) return 0;
		if (!unpack(bf, data.obPoint)) return 0;
		if (!unpack(bf, data.obLine))  return 0;
		if (!unpack(bf, data.obCPoly)) return 0;
					
		// Check that each member has been unpacked
		// and the bytes number mathes what has been specified in data_size
		std::size_t size_read = buffer.size - bf.size;
		return size_read == info.data_size ? (int)info.data_size : 0;
	}
};




// The Global Parser
BinarySerializer binary_ser;
BinaryParser binary_par;

SerializerBase& get_seralizer_binary() { return binary_ser; }
ParserBase& get_parser_binary()        { return binary_par; }
