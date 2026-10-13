#include <metal_stdlib>
using namespace metal;

struct PointVertex {
    float3 position [[attribute(0)]];
    float intensity [[attribute(1)]];
    uint sensorID [[attribute(2)]];
};

struct VertexOut {
    float4 position [[position]];
    float pointSize [[point_size]];
    float4 color;
};

struct Uniforms {
    float4x4 modelViewProjection;
    float pointSize;
    uint colorMode;  // 0=intensity, 1=height, 2=sensor
    float heightMin;
    float heightMax;
};

float3 rainbowGradient(float t) {
    t = saturate(t);
    float3 c;
    if (t < 0.25) {
        c = mix(float3(0.0, 0.0, 1.0), float3(0.0, 1.0, 1.0), t * 4.0);
    } else if (t < 0.5) {
        c = mix(float3(0.0, 1.0, 1.0), float3(0.0, 1.0, 0.0), (t - 0.25) * 4.0);
    } else if (t < 0.75) {
        c = mix(float3(0.0, 1.0, 0.0), float3(1.0, 1.0, 0.0), (t - 0.5) * 4.0);
    } else {
        c = mix(float3(1.0, 1.0, 0.0), float3(1.0, 0.0, 0.0), (t - 0.75) * 4.0);
    }
    return c;
}

vertex VertexOut pointVertex(
    PointVertex in [[stage_in]],
    constant Uniforms& uniforms [[buffer(1)]]
) {
    VertexOut out;
    out.position = uniforms.modelViewProjection * float4(in.position, 1.0);
    out.pointSize = uniforms.pointSize;
    
    float3 color;
    switch (uniforms.colorMode) {
        case 0: {
            // Intensity: grayscale
            float i = saturate(in.intensity);
            color = float3(i, i, i);
            break;
        }
        case 1: {
            // Height: rainbow gradient
            float heightRange = uniforms.heightMax - uniforms.heightMin;
            float normalizedHeight = (heightRange > 0.001)
                ? (in.position.z - uniforms.heightMin) / heightRange
                : 0.5;
            color = rainbowGradient(normalizedHeight);
            break;
        }
        case 2: {
            // Sensor: Mid-360 = cyan, JT128 = magenta
            if (in.sensorID == 0) {
                color = float3(0.0, 0.8, 0.9);  // Cyan for Mid-360
            } else {
                color = float3(0.9, 0.2, 0.8);  // Magenta for JT128
            }
            break;
        }
        default:
            color = float3(1.0, 1.0, 1.0);
    }
    
    out.color = float4(color, 1.0);
    return out;
}

fragment float4 pointFragment(
    VertexOut in [[stage_in]],
    float2 pointCoord [[point_coord]]
) {
    // Circular point shape: pointCoord is [0,1] within the point primitive
    float2 centered = pointCoord - 0.5;
    float dist = length(centered);
    
    if (dist > 0.5) {
        discard_fragment();
    }
    
    // Soft edge falloff for antialiasing
    float alpha = 1.0 - smoothstep(0.35, 0.5, dist);
    return float4(in.color.rgb, in.color.a * alpha);
}
