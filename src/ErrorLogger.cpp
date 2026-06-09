#include <drone_mapper/ErrorLogger.h>

#include <fstream>
#include <iostream>
#include <chrono>
#include <iomanip>

namespace drone_mapper {

std::filesystem::path ErrorLogger::error_log_path;
bool ErrorLogger::initialized = false;

void ErrorLogger::initialize(const std::filesystem::path& filepath) {
    error_log_path = filepath;
    initialized = true;
    
    // Create parent directories if needed
    std::filesystem::create_directories(filepath.parent_path());
    
    // Clear the file and write header
    std::ofstream file(filepath, std::ios::trunc);
    if (file.is_open()) {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        file << "=== Error Log ===" << std::endl;
        file << "Started: " << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S") << std::endl;
        file << "========================================" << std::endl;
        file.close();
    }
}

void ErrorLogger::logError(const std::string& message) {
    if (!initialized) {
        std::cerr << "ErrorLogger not initialized. Error: " << message << std::endl;
        return;
    }

    std::ofstream file(error_log_path, std::ios::app);
    if (file.is_open()) {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        
        file << "[ERROR] " << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
        file << "." << std::setfill('0') << std::setw(3) << ms.count() << " - " << message << std::endl;
        file.close();
    }
    
    // Also print to stderr for visibility
    std::cerr << "[ERROR] " << message << std::endl;
}

void ErrorLogger::logWarning(const std::string& message) {
    if (!initialized) {
        std::cerr << "ErrorLogger not initialized. Warning: " << message << std::endl;
        return;
    }

    std::ofstream file(error_log_path, std::ios::app);
    if (file.is_open()) {
        auto now = std::chrono::system_clock::now();
        auto time = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        
        file << "[WARN] " << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
        file << "." << std::setfill('0') << std::setw(3) << ms.count() << " - " << message << std::endl;
        file.close();
    }
}

std::filesystem::path ErrorLogger::getErrorLogPath() {
    return error_log_path;
}

void ErrorLogger::close() {
    initialized = false;
}

} // namespace drone_mapper
