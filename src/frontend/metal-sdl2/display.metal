#include <metal_stdlib>

using namespace metal;

struct RasterVertex {
  float4 position [[position]];
  float2 texCoord;
};

vertex RasterVertex vertexShader(
  uint vertexID [[vertex_id]],
  constant float2 *vertices [[buffer(0)]],
  constant float2 *viewportSize [[buffer(1)]],
  constant float2 *imageSize [[buffer(2)]]
) {
  float2 pos = vertices[vertexID].xy;

  RasterVertex out;
  out.position = float4(0.0, 0.0, 0.0, 1.0);
  out.position.xy = (pos - 0.5) * 2.0 * (*imageSize) / (*viewportSize);
  out.texCoord = float2(pos.x, 1.0 - pos.y);

  return out;
}

fragment float4 fragmentShader(
  RasterVertex in [[stage_in]],
  texture2d<float, access::sample> textureMap [[texture(0)]],
  sampler textureSampler [[sampler(0)]]
) {
  return textureMap.sample(textureSampler, in.texCoord);
}
