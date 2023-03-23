#include "sparse_block_linear_solver_cholesky.h"

namespace srrg2_solver {

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholesky::updateStructure() {
    assert(_A && _b && " A matrix null");
    _L = SparseBlockCholesky(*_A);
    _x = SparseBlockMatrix(_b->blockRowDims(), _b->blockColDims());
    if (_L.choleskyAllocate()) {
      return SparseBlockLinearSolver::StructureGood;
    }
    return SparseBlockLinearSolver::StructureBad;
    _structure_changed = false;
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholesky::updateCoefficients() {
    _L.setZero();
    _A->copyValues(_L);
    if (_L.choleskyUpdate()) {
      return SparseBlockLinearSolver::CoefficientsGood;
    }
    return SparseBlockLinearSolver::CoefficientsBad;
    _coefficients_changed = false;
  }

  SparseBlockLinearSolver::Status SparseBlockLinearSolverCholesky::updateSolution() {
    _b->copyValues(_x);
    if (_L.choleskySolve(_x)) {
      return SparseBlockLinearSolver::SolutionGood;
    }
    return SparseBlockLinearSolver::SolutionBad;
  }

  bool
  SparseBlockLinearSolverCholesky::computeBlockInverse(SparseBlockMatrix& inverse_blocks,
                                                       const std::vector<IntPair>& blocks_layout_) {
    using namespace std;
    std::vector<IntPair> blocks_layout=blocks_layout_;
    std::sort(blocks_layout.begin(),
              blocks_layout.end(),
              [](const IntPair& a, const IntPair& b)->bool {
                if (a.second<b.second) return true;
                if (a.second>b.second) return false;
                return a.first<b.first;
              });
    
    // fill the matrix with block layout
    for (const IntPair& row_column_idx : blocks_layout) {
      MatrixBlockBase* block =
        inverse_blocks.blockAt(row_column_idx.second, row_column_idx.second, true);
      if (row_column_idx.first == row_column_idx.second) {
        block->setIdentity();
      } else {
        block->setZero();
      }
    }
    bool force_diagonal_dominant=true;
    
    // do block column stuff
    std::vector<float> abs_sums (inverse_blocks.rows(), 0);
    std::vector<float> conditioners (inverse_blocks.rows(), 0);
    for (size_t c=0; c<inverse_blocks._cols.size(); ++c){
      // solve linear system
      SparseBlockMatrix::IntBlockMap& col = inverse_blocks._cols[c];
      if (0 && col.empty())
        continue;
      if (!_L.blockCholeskySolve(col)) {
        return false;
      }
      
      //update the sum of diagonal and non diagonal entries
      if (force_diagonal_dominant) {
        for (auto& it: col) {
          int r=it.first;
          int row_offset=inverse_blocks._row_block_offsets[r];
          int col_offset=inverse_blocks._col_block_offsets[c];
          auto& block=it.second;
          bool is_diagonal = (r==(int) c);
          block->absSum(&(abs_sums[row_offset]), MatrixBlockBase::SumMode::ByRows, is_diagonal);
          block->absSum(&(abs_sums[col_offset]), MatrixBlockBase::SumMode::ByCols, is_diagonal);
          if (is_diagonal) {
            float* cond=&conditioners[row_offset];
            for (int d=0; d<block->rows(); ++d)
              cond[d]=fabs(block->at(d,d));
          }
        }
      }
      //delete non interesting blocks
      for (auto it=col.begin(); it!=col.end(); ++it) {
        int r=it->first;
        IntPair indices(r,c);
        if (!std::binary_search(blocks_layout.begin(),
                               blocks_layout.end(),
                               indices,
                               [](const IntPair& a, const IntPair& b)->bool {
                                 if (a.second<b.second) return true;
                                 if (a.second>b.second) return false;
                                 return a.first<b.first;
                                })){
          auto r_it=it;
          ++it;
          col.erase(r_it);
        }
      }
      
    }
    if (! force_diagonal_dominant)
      return true;
    
    // process the conditioners
    for (size_t i=0; i<conditioners.size(); ++i) {
      float a_ii=conditioners[i];
      float sum_aij=abs_sums[i]/2;
      if (a_ii>=sum_aij)
        conditioners[i]=0;
      else
        conditioners[i]=sum_aij-a_ii;
    }

    // do the shit (trial)
    for (size_t c=0; c<inverse_blocks._cols.size(); ++c){
      // solve linear system
      SparseBlockMatrix::IntBlockMap& col = inverse_blocks._cols[c];
      if (col.empty())
        continue;
      auto it=col.find(c);
      if (it==col.end())
        continue;
      int col_offset=inverse_blocks._col_block_offsets[c];
      MatrixBlockBase* block=it->second.get();
      float* cond=&conditioners[col_offset];
      for (int d=0; d<block->rows(); ++d)
        block->at(d,d)+=cond[d];
    }
    return true;
  }

} // namespace srrg2_solver
