#version 430 core

layout (local_size_x = 64, local_size_y = 1, local_size_z = 1) in;

uniform int batchSize;
uniform float epsSquared;

layout (std430, binding = 0) buffer input_data_t {
    vec4 points[];
} input_data;

layout (std430, binding = 1) buffer output_data_t {
    float distances[];
} output_data;

layout (std430, binding = 2) buffer target_data_t {
    vec4 targets[];
} target_data;

layout (std430, binding = 3) coherent buffer sum_data_t {
    int sums[];
} sum_data;

layout (std430, binding = 4) buffer neighbor_data_t {
    int neighbors[];
} neighbor_data;

void main () {
    ivec2 position = ivec2(gl_GlobalInvocationID.x + gl_LocalInvocationID.x, gl_GlobalInvocationID.y);
    vec3 difference = target_data.targets[position.y].xyz - input_data.points[position.x].xyz;
    vec3 product = difference * difference;
    float squaredDistance = product.x + product.y + product.z;
    output_data.distances[batchSize * position.y + position.x] = squaredDistance;

    if (squaredDistance < epsSquared) {
        int index = atomicAdd(sum_data.sums[position.y], 1);
        neighbor_data.neighbors[batchSize * position.y + index] = position.x;
    }
}
