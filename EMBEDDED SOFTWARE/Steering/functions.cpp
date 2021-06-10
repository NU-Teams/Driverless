

// Returns absolute value of an input
float absoluteValue(float input) {
    if (input < 0) {
        return (-input);
    } else {
        return input;
    }
}

// Returns maximum between two values
float maxValue(float in1, float in2) {
    if (in1 > in2) {
        return in1;
    } else {
        return in2;
    }
}

// Returns minimum between two values
float minValue(float in1, float in2) {
    if (in1 < in2) {
        return in1;
    } else {
        return in2;
    }
}

// Mapping function to map input value with known input limits to some output limits. Must all be floats to accout for divisions.
float mapValue(float input, float min_input, float max_input, float min_output, float max_output) {
    return (min_output + (input - min_input) * (max_output - min_output) / (max_input - min_input));
}