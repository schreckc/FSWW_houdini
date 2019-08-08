#ifndef GRID_HPP
#define GRID_HPP

#include <vector>
#include "definitions.hpp"
#include <SDL2/SDL_image.h>


class Grid {

private:
  int n_rows, n_cols, n_nodes;
  FLOAT cell_size;
  // FLOAT height, width;  //height = n_rows*cell_size and width = n_cols*cell_size
  std::vector<FLOAT> nodes;

  inline int index(int i, int j) const{
    return n_cols*i + j;
  }
  inline int row(int ind) const {
  return ind/n_cols;
}
  inline int col(int ind) const {
  return ind - (row(ind)*n_cols);
}
  
public:
  Grid() {
  n_rows = 0;
  n_cols = 0;
  cell_size = 0;
}
  Grid(int rows, int cols, FLOAT cs = 0.05) {
    n_rows = rows;
    n_cols = cols;
    n_nodes = n_rows*n_cols;
    cell_size = cs;
    nodes = std::vector<FLOAT>(n_nodes);
    reset(0);
  }
  
  ~Grid() {}
  
  bool isEmpty() const {
    return n_nodes == 0;
  }
  
  int getNbRows() const {
    return n_rows;
  }
  int getNbCols() const {
    return n_cols;
  }
    
  FLOAT getCellSize() const {
    return cell_size;
  }
  void setCellSize(FLOAT cs) {
    cell_size = cs;
  }
  float getHeight() const {
   return n_cols*cell_size;
  }
  float getWidth() const {
    return n_rows*cell_size;
  }

  VEC2 toWorld(int i, int j) const {
    VEC2 out(i*cell_size, j*cell_size);
    return out;
  }

  FLOAT operator()(int i, int j) const {
  // assert(ind >= 0 && ind < n_nodes);
    // std::cout<<"i j"<<i<<" "<<j<<std::endl;
  // if (i < 0 || i >= n_rows || j < 0 || j >= n_cols) {
  //   return 0;
  // }
      if (i < 0) {
      i = n_rows+i;
    }
    if (i >= n_rows) {
      i = i - n_rows;
    }
    if (j < 0) {
      j = n_cols+j;
    }
    if (j >= n_cols) {
      j = j - n_cols;
    }

  int ind = index(i, j);
  return nodes[ind];
}

  FLOAT &operator()(int i, int j) {
    // std::cout<<"i --j"<<i<<" "<<j<<std::endl;
  // assert(ind >= 0 && ind < n_nodes);
    //     if (i < 0 || i >= n_rows || j < 0 || j >= n_cols) {
   //   nodes[0] = 0;
  //   return nodes[0];
  // }
    if (i < 0) {
      i = n_rows+i;
    }
    if (i >= n_rows) {
      i = i - n_rows;
    }
    if (j < 0) {
      j = n_cols+j;
    }
    if (j >= n_cols) {
      j = j - n_cols;
    }
   
  int ind = index(i, j);
  return nodes[ind];
}
  
  // FLOAT value(FLOAT x, FLOAT y) const;
  // FLOAT interpolatedValue(FLOAT x, FLOAT y) const;

  void reset(FLOAT val) {
#pragma omp parallel for
    for (int i = 0; i < n_nodes; ++i) {
      nodes[i] = val;
    }
  }

  void setValues(const SDL_Surface *texture) { 
    int w = texture->w;
    int h = texture->h;
  FLOAT pix_per_cell_x = (FLOAT)h/(FLOAT)n_rows;
  FLOAT pix_per_cell_y = (FLOAT)w/(FLOAT)n_cols;
  FLOAT byte_per_cell_x = texture->format->BytesPerPixel * pix_per_cell_x;
  FLOAT byte_per_cell_y = texture->format->BytesPerPixel * pix_per_cell_y;
  unsigned char* pixels = (unsigned char*) texture->pixels;
  
  //  std::cout<<"width height"<<w<<" "<<h<<" "<<n_cols<<" "<<n_rows<<" "<<pix_per_cell_x<<" "<<byte_per_cell_y<<std::endl;
  
  
  #pragma omp parallel for
  for (int i = 0; i < n_rows; ++i) {
    int x = i*byte_per_cell_x/texture->format->BytesPerPixel;//
    x *= texture->format->BytesPerPixel;
    for (int j = 0; j < n_cols; ++j) {
      int y = j*byte_per_cell_y/texture->format->BytesPerPixel;
      y *= texture->format->BytesPerPixel;
      
      nodes[i*n_cols+j] = (int)pixels[(int)(x*w+y)]/256.0;
    }
    //    std::cout<<i<<" "<<x<<" "<<(int)texture->format->BytesPerPixel<<std::endl;

  }
}

  
   Grid& operator=(const Grid& g)  {
  n_rows = g.n_rows;
  n_cols = g.n_cols;
  n_nodes = n_rows*n_cols;
  cell_size = g.cell_size;
  // height = n_rows*cell_size;
  // width = n_cols*cell_size;

  nodes = std::vector<FLOAT>(n_nodes);
  //  #pragma omp for
  for (int i = 0; i < n_nodes; ++i) {
    nodes[i] = g.nodes[i];
  }
  return *this;
}
  
  friend std::ostream& operator<<(std::ostream& os, const Grid& g) {
    os << g.n_rows<<" "<<g.n_cols<<" ";
    for (int i = 0; i < g.n_nodes; ++i) {
      os << g.nodes[i]<<" ";
    }
    return os;
    //  os <<"\n";
  }
  friend std::istream& operator>>(std::istream& is, Grid& g) {
    is >> g.n_rows >> g.n_cols;
    g.n_nodes = g.n_rows*g.n_cols;
    g.nodes = std::vector<FLOAT>(g.n_nodes);
    for (int i = 0; i < g.n_nodes; ++i) {
      is >> g.nodes[i];
  }
    return is;
  }
};

#endif
