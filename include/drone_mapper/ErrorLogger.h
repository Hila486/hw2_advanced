#pragma once

#include <string>
#include <filesystem>

namespace drone_mapper {

/**
 * Handles error and warning logging to file.
 * All errors are logged immediately when they occur, not deferred.
 */
class ErrorLogger {
public:
    /**
     * Initialize the error logger with an output file path.
     * Should be called at the start of the program.
     * @param filepath Path to write error logs to
     */
    static void initialize(const std::filesystem::path& filepath);

    /**
     * Log an error message immediately to the error log file.
     * @param message The error message to log
     */
    static void logError(const std::string& message);

    /**
     * Log a warning message immediately to the error log file.
     * @param message The warning message to log
     */
    static void logWarning(const std::string& message);

    /**
     * Get the current error log file path.
     * @return Path to the error log file
     */
    static std::filesystem::path getErrorLogPath();

    /**
     * Close the error log file.
     */
    static void close();

private:
    static std::filesystem::path error_log_path;
    static bool initialized;
};

} // namespace drone_mapper
