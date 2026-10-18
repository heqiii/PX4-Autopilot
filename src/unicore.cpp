/****************************************************************************
 *
 *   Copyright (c) 2023-2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "unicore.h"
#include "crc.h"
#include <cstdio>
#include <cmath>
#include <stdlib.h>
#include <string.h>

UnicoreParser::Result UnicoreParser::parseChar(char c)
{
	switch (_state) {
	case State::Uninit:
		if (c == '#') {
			_state = State::GotHashtag;
		}

		break;

	case State::GotHashtag:
		if (c == '*') {
			_state = State::GotStar;

			// Make sure buffer is zero terminated.
			_buffer[_buffer_pos] = '\0';

		} else {
			if (_buffer_pos >= sizeof(_buffer) - 1) {
				reset();
				return Result::None;
			}

			_buffer[_buffer_pos++] = c;
		}

		break;

	case State::GotStar:
		_buffer_crc[_buffer_crc_pos++] = c;

		if (_buffer_crc_pos >= 8) {

			// Make sure the CRC buffer is zero terminated.
			_buffer_crc[_buffer_crc_pos] = '\0';

			if (!crcCorrect()) {
				reset();
				return Result::WrongCrc;
			}

			if (isHeading()) {
				if (extractHeading()) {
					reset();
					return Result::GotHeading;

				} else {
					reset();
					return Result::WrongStructure;
				}

			} else if (isAgrica()) {
				if (extractAgrica()) {
					reset();
					return Result::GotAgrica;

				} else {
					reset();
					return Result::WrongStructure;
				}

			} else {
				reset();
				return Result::UnknownSentence;
			}
		}

		break;
	}

	return Result::None;
}

void UnicoreParser::reset()
{
	_state = State::Uninit;
	_buffer_pos = 0;
	_buffer_crc_pos = 0;
}

bool UnicoreParser::crcCorrect() const
{
	const uint32_t crc_calculated = calculateCRC32(_buffer_pos, (uint8_t *)_buffer, 0);
	const uint32_t crc_parsed = (uint32_t)strtoul(_buffer_crc, nullptr, 16);
	return (crc_calculated == crc_parsed);
}

bool UnicoreParser::isHeading() const
{
	const char header[] = "UNIHEADINGA";

	return strncmp(header, _buffer, strlen(header)) == 0 && _buffer[strlen(header)] == 44;
}

bool UnicoreParser::isAgrica() const
{
	const char header[] = "AGRICA";

	return strncmp(header, _buffer, strlen(header)) == 0 && _buffer[strlen(header)] == 44;
}

bool UnicoreParser::extractHeading()
{
    _heading = {};
    char *body = strchr(_buffer, ';');
    if (!body) { return false; }
    char *fields[8] {};
    fields[0] = body + 1;
    for (unsigned i = 1; i < 8; ++i) {
        char *comma = strchr(fields[i - 1], ',');
        if (!comma) { return false; }
        *comma = '\0'; fields[i] = comma + 1;
    }
    if (char *comma = strchr(fields[7], ',')) { *comma = '\0'; }
    float values[3] {};
    const unsigned indices[] = {2, 3, 6};
    for (unsigned i = 0; i < 3; ++i) {
        char *end = nullptr;
        values[i] = strtof(fields[indices[i]], &end);
        if (end == fields[indices[i]] || *end != '\0' || !std::isfinite(values[i])) { return false; }
    }
    _heading.baseline_m = values[0];
    _heading.heading_deg = values[1];
    _heading.heading_stddev_deg = values[2];
    const bool fixed = strcmp(fields[1], "NARROW_INT") == 0 || strcmp(fields[1], "L1_INT") == 0
                       || strcmp(fields[1], "WIDE_INT") == 0;
    _heading.valid = strcmp(fields[0], "SOL_COMPUTED") == 0 && fixed
                     && values[0] > 0.f && values[1] >= 0.f && values[1] < 360.f
                     && values[2] > 0.f && values[2] <= 180.f;
    return true;
}

bool UnicoreParser::extractAgrica()
{
    _agrica_valid = false;
    char *body = strchr(_buffer, ';');
    if (!body) { return false; }
    char *field = body + 1;
    float velocity[7] {};
    for (unsigned index = 0; index <= 28; ++index) {
        char *comma = strchr(field, ',');
        if (!comma) { return false; }
        *comma = '\0';
        if (index >= 22) {
            char *end = nullptr;
            velocity[index - 22] = strtof(field, &end);
            if (end == field || *end != '\0' || !std::isfinite(velocity[index - 22])) { return false; }
        }
        field = comma + 1;
    }
    if (velocity[0] < 0.f || velocity[4] < 0.f || velocity[5] < 0.f || velocity[6] < 0.f) { return false; }
    _agrica.velocity_m_s = velocity[0];
    _agrica.velocity_north_m_s = velocity[1];
    _agrica.velocity_east_m_s = velocity[2];
    _agrica.velocity_up_m_s = velocity[3];
    _agrica.stddev_velocity_north_m_s = velocity[4];
    _agrica.stddev_velocity_east_m_s = velocity[5];
    _agrica.stddev_velocity_up_m_s = velocity[6];
    _agrica_valid = true;
    return true;
}
