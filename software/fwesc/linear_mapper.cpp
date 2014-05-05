/**
* @author Alexander Entinger, BSc
* @brief this file implements a linear mapper for adapting signal ranges between various functions
* @file linear_mapper.cpp
* @license CC BY-NC-SA 3.0 ( http://creativecommons.org/licenses/by-nc-sa/3.0/ )
*/

#include "linear_mapper.h"

/**
 * @brief initializes the linear mapper
 * @param lm the linear mapper ADT (abstract data type)
 * @param input_min minimum value of the input
 * @param input_max maximum value of the input
 * @param output_min minimum output value
 * @param output_max maximum output value
 */
linear_mapper::linear_mapper(int16_t const input_min, int16_t const input_max, int16_t const output_min, int16_t const output_max) {
	this->init(input_min, input_max, output_min, output_max);
}

/** 
 * @brief reinitializes this linear mapper
 */
void linear_mapper::init(int16_t const input_min, int16_t const input_max, int16_t const output_min, int16_t const output_max) {
	int32_t const delta_input = ((int32_t)(input_max) - (int32_t)(input_min)) << 8;
	int32_t const delta_output = ((int32_t)(output_max) - (int32_t)(output_min)) << 8;
		
	m_k =	(int32_t)
			(
				delta_output
				/
				delta_input
			); // multiplication by 16 for increasing accuracy in the mapping function
		
	m_d =	(int32_t)
			(
				(int32_t)(output_max)
				-
				(
					m_k * (int32_t)(input_max)
				)
			);
}

/**
 * @brief performs the linear mapping 
 * @param value input value to be mapped to output value
 * @return mapped output value
 */
int16_t linear_mapper::map(int16_t const value) {
	return (int16_t)
			(
			(m_k * (int32_t)(value))
			+
			m_d
			);
}