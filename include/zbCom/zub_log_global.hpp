/*!
 * This file is part of the ZbCom library which provides basic cross-compatible communication
 * functionality for interfacing a MACS controller. For full license details see the ZbCom
 * library.
 *
 * Copyright: zub machine control AG
 */

#pragma once

#include <string>
#include <vector>

enum log_level_t {
    log_level_trace,
    log_level_debug,
    log_level_info,
    log_level_warning,
    log_level_error,
    log_level_fatal
};

/*!
 * \brief converts a byte vector to a string in hexadecimal format
 * \param byte_vector the byte vector to be converted
 * \return the string with a whitespace separated hexadecimal numbers from the vector
 */
std::string to_string_hex(std::vector<uint8_t> byte_vector);

/*!
 * \brief enables the log
 * \param log_level for controlling the amount of log that is outputted every log message has a
 *                  level assigned, this parameter controls what level of information is written to
 *                  the log 
 */
void enable_log(log_level_t log_level);

/*!
 * \brief disables the log
 */
void disable_log();
