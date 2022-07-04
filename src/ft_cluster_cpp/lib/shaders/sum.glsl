#version 460 core

layout (binding = 3) buffer input_data_t {
    int toSum[];
} input_data;

layout (binding = 4) buffer output_data_t {
    int sum[];
} output_data;

void main() {
    int index = int(gl_GlobalInvocationID.x);
    output_data.sum[index] = input_data.otoSum[index] + input_data.toSum[index+1];
}
