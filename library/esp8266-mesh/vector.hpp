#pragma once

#include <functional>

template<typename T>
class vector {
public:
	vector(uint32_t startLength = 0)
		:	elementLength{startLength} {

		elementSize = startLength;
		if(elementSize == 0) {
			elementSize = 1;
		}
		else {
			elementSize--;
			elementSize |= elementSize >> 1;
			elementSize |= elementSize >> 2;
			elementSize |= elementSize >> 4;
			elementSize |= elementSize >> 8;
			elementSize |= elementSize >> 16;
			elementSize++;
		}


		elements = reinterpret_cast<T*>(malloc(elementSize * sizeof(T)));

		for(uint32_t i = 0; i < startLength; ++i) {
			new(elements + i) T{};
		}
	}

	~vector() {
		free(elements);
	}

	const T& operator[](size_t i) const {
		return elements[i];
	}

	T& operator[](size_t i) {
		return elements[i];
	}

	void push_back(const T& element) {
		if(elementLength == elementSize) {
			elementSize <<= 1;
			
			elements = reinterpret_cast<T*>(realloc(elements, elementSize * sizeof(T)));

			if(elements == nullptr) {
				std::cerr << "[Error] vector::push_back: realloc failed" << std::endl;
			}
		}

		new(elements + elementLength) T{element};
		elementLength++;
	}

	const T* data() const {
		return elements;
	}

	T* data() {
		return elements;
	}

	void clear() {
		elementLength = 0;
	}

	void erase(size_t i) {
		if(i != (elementLength-1)) {
			for(size_t j = i; j < (elementLength-1); ++j) {
				elements[j] = elements[j+1];
			}
		}
		
		elementLength--;
	}

	size_t size() const {
		return elementLength;
	}

	bool empty() const {
		return elementLength == 0;
	}

private:
	size_t elementSize;
	size_t elementLength;
	T* elements;
};


template<typename T>
size_t find_index_if(const vector<T>& v,
	const std::function<bool(const T&)>& condition) {
	size_t i;

	for(i = 0; i < v.size(); ++v) {
		if(condition(v[i])) {
			break;
		}
	}

	return i;
}
