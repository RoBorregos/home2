const path = require('path');

module.exports = {
  entry: './src/index.tsx',
  output: {
    path: path.resolve(__dirname, 'public'),
    filename: 'bundle.js',
  },
  resolve: {
    extensions: ['.tsx', '.ts', '.js'],
  },
  module: {
    rules: [
      {
        test: /\.tsx?$/,
        use: 'ts-loader',
        exclude: /node_modules/,
      },
      {
        test: /\.css$/, // Handles CSS files
        use: [
          'style-loader', // Injects CSS into the DOM
          'css-loader', // Resolves CSS imports and URLs
          'postcss-loader', // Uses PostCSS to process the CSS
        ],
      },
    ],
  },
  devServer: {
    contentBase: path.join(__dirname, 'dist'),
    hot: true,
    port: 3001,
  },
};
