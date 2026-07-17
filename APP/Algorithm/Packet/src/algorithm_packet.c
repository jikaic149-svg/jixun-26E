#include "algorithm_packet.h"

#include <limits.h>

static bool parser_is_valid(const Algorithm_Packet_Parser *parser)
{
    return (parser != NULL) && (parser->storage != NULL) &&
        (parser->storage_capacity >= ALGORITHM_PACKET_PARSER_STORAGE_SIZE);
}

static void packet_clear(Algorithm_Packet_View *packet)
{
    packet->type = NULL;
    packet->type_length = 0U;
    packet->sequence = 0U;
    packet->payload = NULL;
    packet->payload_length = 0U;
}

static uint32_t increment_saturated(uint32_t value)
{
    return (value == UINT32_MAX) ? UINT32_MAX : (value + 1U);
}

static size_t decimal_digit_count(size_t value)
{
    size_t digits = 1U;

    while (value >= 10U) {
        value /= 10U;
        ++digits;
    }
    return digits;
}

static size_t write_decimal(uint8_t *output, size_t value)
{
    const size_t digits = decimal_digit_count(value);
    size_t index = digits;

    do {
        --index;
        output[index] = (uint8_t) ('0' + (value % 10U));
        value /= 10U;
    } while (index > 0U);

    return digits;
}

static bool type_is_valid(const char *type, size_t *type_length)
{
    size_t length = 0U;

    if (type == NULL) {
        return false;
    }
    while ((length <= ALGORITHM_PACKET_MAX_TYPE_LENGTH) &&
        (type[length] != '\0')) {
        const unsigned char byte = (unsigned char) type[length];

        if (length == 0U) {
            if ((byte < (unsigned char) 'A') ||
                (byte > (unsigned char) 'Z')) {
                return false;
            }
        }
        else if (!(((byte >= (unsigned char) 'A') &&
                       (byte <= (unsigned char) 'Z')) ||
                     ((byte >= (unsigned char) '0') &&
                       (byte <= (unsigned char) '9')) ||
                     (byte == (unsigned char) '_'))) {
            return false;
        }
        ++length;
    }

    if ((length == 0U) || (length > ALGORITHM_PACKET_MAX_TYPE_LENGTH)) {
        return false;
    }
    *type_length = length;
    return true;
}

static bool payload_is_valid(const uint8_t *payload, size_t payload_length)
{
    size_t index;

    if ((payload_length > 0U) && (payload == NULL)) {
        return false;
    }
    for (index = 0U; index < payload_length; ++index) {
        if ((payload[index] < 0x20U) || (payload[index] > 0x7EU)) {
            return false;
        }
    }
    return true;
}

static size_t find_separator(
    const uint8_t *line, size_t start, size_t line_length)
{
    size_t index;

    for (index = start; index < line_length; ++index) {
        if (line[index] == (uint8_t) '|') {
            return index;
        }
    }
    return SIZE_MAX;
}

static bool parse_decimal(
    const uint8_t *bytes,
    size_t length,
    uint32_t maximum,
    uint32_t *value)
{
    uint32_t parsed = 0U;
    size_t index;

    if ((length == 0U) || ((length > 1U) && (bytes[0] == (uint8_t) '0'))) {
        return false;
    }
    for (index = 0U; index < length; ++index) {
        const uint8_t byte = bytes[index];
        uint32_t digit;

        if ((byte < (uint8_t) '0') || (byte > (uint8_t) '9')) {
            return false;
        }
        digit = (uint32_t) (byte - (uint8_t) '0');
        if ((parsed > (maximum / 10U)) ||
            ((parsed == (maximum / 10U)) &&
                (digit > (maximum % 10U)))) {
            return false;
        }
        parsed = (parsed * 10U) + digit;
    }
    *value = parsed;
    return true;
}

static bool parse_crc_hex(const uint8_t *bytes, uint16_t *value)
{
    uint16_t parsed = 0U;
    size_t index;

    for (index = 0U; index < 4U; ++index) {
        const uint8_t byte = bytes[index];
        uint16_t nibble;

        if ((byte >= (uint8_t) '0') && (byte <= (uint8_t) '9')) {
            nibble = (uint16_t) (byte - (uint8_t) '0');
        }
        else if ((byte >= (uint8_t) 'A') && (byte <= (uint8_t) 'F')) {
            nibble = (uint16_t) ((byte - (uint8_t) 'A') + 10U);
        }
        else {
            return false;
        }
        parsed = (uint16_t) ((uint16_t) (parsed << 4U) | nibble);
    }
    *value = parsed;
    return true;
}

static Algorithm_Packet_Error parse_line(
    const uint8_t *line,
    size_t line_length,
    Algorithm_Packet_View *packet)
{
    size_t type_end;
    size_t sequence_start;
    size_t sequence_end;
    size_t length_start;
    size_t length_end;
    size_t payload_start;
    size_t payload_length;
    size_t crc_separator;
    uint32_t sequence;
    uint32_t declared_length;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((line_length == 0U) || (line[0] != (uint8_t) '@')) {
        return ALGORITHM_PACKET_ERROR_INVALID_START;
    }
    if ((line_length < 3U) || (line[1] != (uint8_t) '1')) {
        return ALGORITHM_PACKET_ERROR_UNSUPPORTED_VERSION;
    }
    if (line[2] != (uint8_t) '|') {
        return ALGORITHM_PACKET_ERROR_INVALID_FORMAT;
    }

    type_end = find_separator(line, 3U, line_length);
    if (type_end == SIZE_MAX) {
        return ALGORITHM_PACKET_ERROR_INVALID_FORMAT;
    }
    if ((type_end == 3U) ||
        ((type_end - 3U) > ALGORITHM_PACKET_MAX_TYPE_LENGTH)) {
        return ALGORITHM_PACKET_ERROR_INVALID_TYPE;
    }
    {
        size_t index;

        for (index = 3U; index < type_end; ++index) {
            const uint8_t byte = line[index];

            if (index == 3U) {
                if ((byte < (uint8_t) 'A') || (byte > (uint8_t) 'Z')) {
                    return ALGORITHM_PACKET_ERROR_INVALID_TYPE;
                }
            }
            else if (!(((byte >= (uint8_t) 'A') &&
                           (byte <= (uint8_t) 'Z')) ||
                         ((byte >= (uint8_t) '0') &&
                           (byte <= (uint8_t) '9')) ||
                         (byte == (uint8_t) '_'))) {
                return ALGORITHM_PACKET_ERROR_INVALID_TYPE;
            }
        }
    }

    sequence_start = type_end + 1U;
    sequence_end = find_separator(line, sequence_start, line_length);
    if (sequence_end == SIZE_MAX) {
        return ALGORITHM_PACKET_ERROR_INVALID_FORMAT;
    }
    if (!parse_decimal(&line[sequence_start],
            sequence_end - sequence_start, UINT16_MAX, &sequence)) {
        return ALGORITHM_PACKET_ERROR_INVALID_SEQUENCE;
    }

    length_start = sequence_end + 1U;
    length_end = find_separator(line, length_start, line_length);
    if (length_end == SIZE_MAX) {
        return ALGORITHM_PACKET_ERROR_INVALID_FORMAT;
    }
    if (!parse_decimal(&line[length_start], length_end - length_start,
            ALGORITHM_PACKET_MAX_PAYLOAD_LENGTH, &declared_length)) {
        return ALGORITHM_PACKET_ERROR_INVALID_LENGTH;
    }

    payload_start = length_end + 1U;
    payload_length = (size_t) declared_length;
    if ((payload_start > line_length) ||
        (payload_length > (line_length - payload_start))) {
        return ALGORITHM_PACKET_ERROR_INVALID_LENGTH;
    }
    crc_separator = payload_start + payload_length;
    if ((crc_separator >= line_length) ||
        ((line_length - crc_separator) != 5U) ||
        (line[crc_separator] != (uint8_t) '|')) {
        return ALGORITHM_PACKET_ERROR_INVALID_LENGTH;
    }
    if (!payload_is_valid(&line[payload_start], payload_length)) {
        return ALGORITHM_PACKET_ERROR_INVALID_PAYLOAD;
    }
    if (!parse_crc_hex(&line[crc_separator + 1U], &received_crc)) {
        return ALGORITHM_PACKET_ERROR_INVALID_FORMAT;
    }

    calculated_crc = Algorithm_Packet_Crc16CcittFalse(
        line, crc_separator + 1U);
    if (calculated_crc != received_crc) {
        return ALGORITHM_PACKET_ERROR_CRC_MISMATCH;
    }

    packet->type = &line[3];
    packet->type_length = type_end - 3U;
    packet->sequence = (uint16_t) sequence;
    packet->payload = &line[payload_start];
    packet->payload_length = payload_length;
    return ALGORITHM_PACKET_ERROR_NONE;
}

static Algorithm_Packet_FeedResult reject_frame(
    Algorithm_Packet_Parser *parser,
    Algorithm_Packet_Error error,
    bool discard_until_lf)
{
    parser->length = 0U;
    parser->error_count = increment_saturated(parser->error_count);
    parser->last_error = error;
    parser->discarding = discard_until_lf;
    return ALGORITHM_PACKET_FEED_FRAME_REJECTED;
}

uint16_t Algorithm_Packet_Crc16CcittFalse(
    const uint8_t *bytes, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t byte_index;

    for (byte_index = 0U; byte_index < length; ++byte_index) {
        unsigned int bit_index;

        crc ^= (uint16_t) ((uint16_t) bytes[byte_index] << 8U);
        for (bit_index = 0U; bit_index < 8U; ++bit_index) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t) ((uint16_t) (crc << 1U) ^ 0x1021U);
            }
            else {
                crc = (uint16_t) (crc << 1U);
            }
        }
    }
    return crc;
}

Algorithm_Packet_EncodeResult Algorithm_Packet_Encode(
    const char *type,
    uint16_t sequence,
    const uint8_t *payload,
    size_t payload_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    static const uint8_t hex_digits[] = "0123456789ABCDEF";
    size_t type_length;
    size_t sequence_digits;
    size_t length_digits;
    size_t required_size;
    size_t position = 0U;
    size_t index;
    uint16_t crc;

    if (output_length == NULL) {
        return ALGORITHM_PACKET_ENCODE_INVALID_ARGUMENT;
    }
    *output_length = 0U;
    if (!type_is_valid(type, &type_length)) {
        return ALGORITHM_PACKET_ENCODE_INVALID_TYPE;
    }
    if ((payload_length > ALGORITHM_PACKET_MAX_PAYLOAD_LENGTH) ||
        !payload_is_valid(payload, payload_length)) {
        return ALGORITHM_PACKET_ENCODE_INVALID_PAYLOAD;
    }

    sequence_digits = decimal_digit_count((size_t) sequence);
    length_digits = decimal_digit_count(payload_length);
    required_size = 3U + type_length + 1U + sequence_digits + 1U +
        length_digits + 1U + payload_length + 1U + 4U + 1U;
    if (required_size > ALGORITHM_PACKET_MAX_FRAME_SIZE) {
        return ALGORITHM_PACKET_ENCODE_FRAME_TOO_LONG;
    }
    if (output_capacity < required_size) {
        *output_length = required_size;
        return ALGORITHM_PACKET_ENCODE_OUTPUT_TOO_SMALL;
    }
    if (output == NULL) {
        return ALGORITHM_PACKET_ENCODE_INVALID_ARGUMENT;
    }

    output[position++] = (uint8_t) '@';
    output[position++] = (uint8_t) '1';
    output[position++] = (uint8_t) '|';
    for (index = 0U; index < type_length; ++index) {
        output[position++] = (uint8_t) (unsigned char) type[index];
    }
    output[position++] = (uint8_t) '|';
    position += write_decimal(&output[position], (size_t) sequence);
    output[position++] = (uint8_t) '|';
    position += write_decimal(&output[position], payload_length);
    output[position++] = (uint8_t) '|';
    for (index = 0U; index < payload_length; ++index) {
        output[position++] = payload[index];
    }
    output[position++] = (uint8_t) '|';

    crc = Algorithm_Packet_Crc16CcittFalse(output, position);
    output[position++] = hex_digits[(crc >> 12U) & 0x0FU];
    output[position++] = hex_digits[(crc >> 8U) & 0x0FU];
    output[position++] = hex_digits[(crc >> 4U) & 0x0FU];
    output[position++] = hex_digits[crc & 0x0FU];
    output[position++] = (uint8_t) '\n';

    *output_length = position;
    return ALGORITHM_PACKET_ENCODE_OK;
}

bool Algorithm_Packet_Parser_Init(
    Algorithm_Packet_Parser *parser,
    uint8_t *storage,
    size_t storage_capacity)
{
    if (parser == NULL) {
        return false;
    }

    parser->storage = NULL;
    parser->storage_capacity = 0U;
    parser->length = 0U;
    parser->accepted_count = 0U;
    parser->error_count = 0U;
    parser->last_error = ALGORITHM_PACKET_ERROR_NONE;
    parser->discarding = false;

    if ((storage == NULL) ||
        (storage_capacity < ALGORITHM_PACKET_PARSER_STORAGE_SIZE)) {
        return false;
    }
    parser->storage = storage;
    parser->storage_capacity = storage_capacity;
    return true;
}

void Algorithm_Packet_Parser_Reset(Algorithm_Packet_Parser *parser)
{
    if (!parser_is_valid(parser)) {
        return;
    }
    parser->length = 0U;
    parser->accepted_count = 0U;
    parser->error_count = 0U;
    parser->last_error = ALGORITHM_PACKET_ERROR_NONE;
    parser->discarding = false;
}

Algorithm_Packet_FeedResult Algorithm_Packet_Parser_Feed(
    Algorithm_Packet_Parser *parser,
    uint8_t byte,
    Algorithm_Packet_View *packet)
{
    Algorithm_Packet_Error error;
    size_t line_length;

    if (!parser_is_valid(parser) || (packet == NULL)) {
        return ALGORITHM_PACKET_FEED_INVALID_ARGUMENT;
    }
    packet_clear(packet);

    if (parser->discarding) {
        if (byte == (uint8_t) '\n') {
            parser->discarding = false;
        }
        return ALGORITHM_PACKET_FEED_IN_PROGRESS;
    }

    if (byte == (uint8_t) '\n') {
        line_length = parser->length;
        if ((line_length > 0U) &&
            (parser->storage[line_length - 1U] == (uint8_t) '\r')) {
            --line_length;
        }
        error = parse_line(parser->storage, line_length, packet);
        parser->length = 0U;
        if (error != ALGORITHM_PACKET_ERROR_NONE) {
            return reject_frame(parser, error, false);
        }
        parser->accepted_count = increment_saturated(parser->accepted_count);
        parser->last_error = ALGORITHM_PACKET_ERROR_NONE;
        return ALGORITHM_PACKET_FEED_FRAME_READY;
    }

    if (parser->length == 0U) {
        if (byte != (uint8_t) '@') {
            return reject_frame(
                parser, ALGORITHM_PACKET_ERROR_INVALID_START, true);
        }
    }
    if (parser->length >= ALGORITHM_PACKET_PARSER_STORAGE_SIZE) {
        return reject_frame(
            parser, ALGORITHM_PACKET_ERROR_FRAME_TOO_LONG, true);
    }
    parser->storage[parser->length++] = byte;
    return ALGORITHM_PACKET_FEED_IN_PROGRESS;
}

uint32_t Algorithm_Packet_Parser_AcceptedCount(
    const Algorithm_Packet_Parser *parser)
{
    return parser_is_valid(parser) ? parser->accepted_count : 0U;
}

uint32_t Algorithm_Packet_Parser_ErrorCount(
    const Algorithm_Packet_Parser *parser)
{
    return parser_is_valid(parser) ? parser->error_count : 0U;
}

Algorithm_Packet_Error Algorithm_Packet_Parser_LastError(
    const Algorithm_Packet_Parser *parser)
{
    return parser_is_valid(parser) ? parser->last_error :
        ALGORITHM_PACKET_ERROR_NONE;
}
