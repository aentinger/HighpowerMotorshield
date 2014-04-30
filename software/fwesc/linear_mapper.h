/**
* @author Alexander Entinger, BSc
* @brief this file implements a linear mapper for adapting signal ranges between various functions
* @file linear_mapper.h
*/

#ifndef LINEAR_MAPPER_H_
#define LINEAR_MAPPER_H_

#include <stdint.h>

class linear_mapper {
public:
	/**
	 * @brief Constructor initializes the linear mapper
	 * @param lm the linear mapper ADT (abstract data type)
	 * @param input_min minimum value of the input
	 * @param input_max maximum value of the input
	 * @param output_min minimum output value
	 * @param output_max maximum output value
	 */
	linear_mapper(int16_t const input_min, int16_t const input_max, int16_t const output_min, int16_t const output_max);

	/**
	* @brief performs the linear mapping 
	* @param value input value to be mapped to output value
	* @return mapped output value
	*/
	int16_t map(int16_t const value);

private:
	int32_t m_k;
	int32_t m_d;
};

#endif /* LINEAR_MAPPER_H_ */