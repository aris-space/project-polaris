// Webpack customisation picked up by `foxglove-extension build`.
//
// Kept as plain CommonJS JavaScript (no type annotations) so Node can load it
// on any version without ts-node.
//
// leaflet.css references marker/layer-control PNGs via url(). We don't use
// those (all markers are SVG divIcons), and the stock extension webpack config
// has no image loader, so disable css-loader url() resolution.
module.exports = {
  webpack: (config) => {
    for (const rule of config.module.rules) {
      if (rule && rule.test instanceof RegExp && rule.test.test("x.css")) {
        rule.use = ["style-loader", { loader: "css-loader", options: { url: false } }];
      }
    }
    return config;
  },
};
