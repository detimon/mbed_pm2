#ifndef AVG_FILTER_H
#define AVG_FILTER_H

#include "mbed.h"

class AvgFilter {
public:
    AvgFilter(int size = 10) : _size(size), _index(0), _sum(0.0) {
        _buffer = new float[_size]();
    }
    ~AvgFilter() { delete[] _buffer; }

    float add(float val) {
        _sum -= _buffer[_index];
        _buffer[_index] = val;
        _sum += val;
        _index = (_index + 1) % _size;
        return _sum / _size;
    }

private:
    int _size, _index;
    float *_buffer;
    float _sum;
};

#endif