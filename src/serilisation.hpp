#ifndef SERIALISATION_HPP
#define SERIALISATION_HPP

#include <fstream>
#include <filesystem>
#include <vector>

namespace core {

struct binary_reader_t {
    binary_reader_t(const std::filesystem::path& path) : _path(path), _file(path, std::ios::binary) {
        if (!_file.is_open()) {
            throw std::runtime_error("Failed to open file");
        }
    }

    ~binary_reader_t() {
        _file.close();
    }

    size_t file_size() {
        return std::filesystem::file_size(_path);
    }
    
    // TODO: add error handling
    template <typename type_t>
    void read(type_t& val) {
        _file.read(reinterpret_cast<char *>(&val), sizeof(type_t));
    }

    std::filesystem::path   _path;
    std::ifstream           _file;
};

struct binary_writer_t {
    binary_writer_t(const std::filesystem::path& path) : _path(path), _file(path, std::ios::binary) {
        if (!_file.is_open()) {
            throw std::runtime_error("Failed to open file");
        }
    }

    ~binary_writer_t() {
        flush();
        _file.close();
    }

    template <typename type_t>
    void write(const type_t& val) {
        const char *data = reinterpret_cast<const char *>(&val);
        for (size_t i = 0; i < sizeof(type_t); i++) {
            _buffer.push_back(data[i]);
        }
    }

    void flush() {
        _file.write(_buffer.data(), _buffer.size());
        _buffer.clear();
    }

    std::filesystem::path   _path;
    std::ofstream           _file;
    std::vector<char>       _buffer;
};

} // namespace core

#endif