#pragma once
#include "factor_base.h"
#include <srrg_boss/serializer.h>
#include <srrg_data_structures/abstract_ptr_map.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  /*!@brief  General interface that defines the basic functionality to manage a factor graph.
    In the derived class you must specify the accessors to the id to factors and id to variables
    containers.
   */
  class FactorGraphInterface {
    struct VariableFactorPair: public std::pair<VariableBase::Id, FactorBase*> {
      VariableFactorPair(VariableBase::Id id, FactorBase* f):
        std::pair<VariableBase::Id, FactorBase*>(id,f){};
      inline bool operator<(const VariableFactorPair& other) {
        if (first<other.first)
          return true;
        if (first<other.first)
          return false;
        if (second && other.second)
          return second->graphId()<other.second->graphId();
        return second<other.second;
        return false;
      }
    };
  public:
    using VariableFactorMap         = std::set<VariableFactorPair>;
    using VariableFactorMapIterator = VariableFactorMap::iterator;
    using Id                        = int64_t;
    /*! Accessor for a variable using the graph id
      @param[in] id the graph id of the variable
      @return The variable pointer
     */
    VariableBase* variable(VariableBase::Id id);
    /*! Accessor for a factor using the graph id
      @param[in] id the graph id of the factor
      @return The factor pointer
     */
    FactorBase* factor(FactorBase::Id id);
    void printVariables();
    virtual ~FactorGraphInterface();
    /*! @return The container of variable pointers  */
    virtual IdVariablePtrContainer& variables() = 0;
    /*! @return The container of factor pointers  */
    virtual IdFactorPtrContainer& factors() = 0;

    virtual void clear();

    VariableFactorMap& factors(const VariableBase* v); // returns the factors of this variable
    VariableFactorMap::iterator lowerFactor(const VariableBase* v) {return factors(v).begin();}
    VariableFactorMap::iterator upperFactor(const VariableBase* v) {return factors(v).end();}
    
    virtual void addVariable(VariableBase* v) = 0;
    virtual void addFactor(FactorBase* f) = 0;
    virtual bool removeVariable(VariableBase* v);
    virtual bool removeFactor(FactorBase* f);

    void write(const std::string& filename);

  protected:
    /*! Connect a factor with the corresponding variables
      @return Number of variables correctly connected
     */
    virtual int bindFactor(FactorBase* factor);
    /*! Disconnect a factor from the corresponding variables
     */
    virtual void unbindFactor(FactorBase* factor);
    /*! Call bindFactor() for each factor in the interface
      @return Total number of variables connected
    */
    virtual int bindFactors();
    /*! Remove all the factors and variables from the interface */
    
    std::map<VariableBase::Id, VariableFactorMap> _var_to_factor_map;
    
    void _write(srrg2_core::Serializer& ser, std::set<FactorBase*>& factors);

    VariableFactorMap& _v2f(VariableBase::Id id, bool insert=false);
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using FactorGraphInterfacePtr =
    std::shared_ptr<FactorGraphInterface>; /*!< Shared pointer
                                             to FactorGraphInterface */

  /*! @brief View of a factor graph, does not have ownership of variables and factors.
   As such the accessors to the id to variables and id to factors container have a raw pointer
   as value. The compatibility among different container types (same key type different value type)
   is achieved through AbstractMap_, see srrg2_core.
   */
  class FactorGraphView : public FactorGraphInterface {
  public:
    using IdVariableRawPtrMap = AbstractMap_<VariableBase::Id, VariableBase*>;
    using IdFactorRawPtrMap   = AbstractMap_<FactorBase::Id, FactorBase*>;
    virtual ~FactorGraphView();
    /*! @return The container of variable pointers*/
    IdVariablePtrContainer& variables() override;
    /*! @return The container of factor pointers*/
    IdFactorPtrContainer& factors() override;

    void addVariable(VariableBase* v) override;
    void addFactor(FactorBase* f) override;
    bool removeVariable(VariableBase* v) override;
    bool removeFactor(FactorBase* f) override;

    // makes the union between this view and another one
    void add(FactorGraphInterface& src, int level=-1);
    // makes the union between this view and another one,
    // selecting the factors with proper id from src.
    
    void addFactors(FactorGraphInterface& src, const std::set<FactorBase::Id>& factors);

    // makes the union between this view and another one, selecting the variables in the src,
    // and all factors connected, that join variables in the set
    void addVariables(FactorGraphInterface& src, const std::set<VariableBase::Id>&, int level=-1);
    
  protected:
    IdVariableRawPtrMap _variables; /*!< Id to variable raw pointer container */
    IdFactorRawPtrMap _factors;     /*!< Id to factor raw pointer container */

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using FactorGraphViewPtr = std::shared_ptr<FactorGraphView>; /*!<Shared pointer
                                                                 to FactorGraphView */

  /*! @brief Materialization of a factor graph. A factor graph *owns* its variables and factors, so
    in this case the id to variable and id to factors container have as value a shared pointer.

    An example of factor graph interface without ownership is FactorGraphView.
  */
  class FactorGraph : public FactorGraphInterface, public Serializable {
  public:
    /*! Add a variable to the factor graph
      @param[in] var a shared pointer to variable
      @return false if the variable was already present in the graph
    */
    bool addVariable(VariableBasePtr var);
    /*! Add a factor to the graph
      @param[in] factor a shared pointer to factor
      @return false if the factor was already present in the graph
    */
    bool addFactor(FactorBasePtr factor);
    /*! Remove factor from the graph
      @param[in] factor a shared pointer to factor
      @return false if the factor was not in the graph
    */
    bool removeFactor(FactorBasePtr factor);
    /*! Auxiliary function used to remove a factor from the graph using a raw pointer
      @param[in] factor raw pointer to factor
      @return false if the factor was not present in the graph
    */
    bool removeFactor(FactorBase* factor);

    bool removeVariable(VariableBasePtr var);

    bool removeVariable(VariableBase* var);

    virtual ~FactorGraph();

    IdVariablePtrContainer& variables() override;
    IdFactorPtrContainer& factors() override;

    void serialize(ObjectData& odata, IdContext& context) override;
    void deserialize(ObjectData& odata, IdContext& context) override;


    /*! Read a graph from a file (static method)
      @param[in] filename
      @return shared pointer to the factor graph loaded
    */
    static std::shared_ptr<FactorGraph> read(const std::string& filename);
    /*! @return Last graph id*/
    const Id& lastGraphId() const {
      return _last_graph_id;
    }

    /*! Write a factor graph on a file
      @param[in] filename
    */
    void write(const std::string& filename);

    VariableBasePtr detachVariable(VariableBase* v);
    FactorBasePtr detachFactor(FactorBase* f);

    void setSerializationLevel(int level_) {_level_serialization=level_;}
  protected:
    using IdVariablePtrMap =
      AbstractPtrMap_<VariableBase::Id, VariableBase, std::shared_ptr<VariableBase>>;
    using IdFactorPtrMap = AbstractPtrMap_<FactorBase::Id, FactorBase, std::shared_ptr<FactorBase>>;
    IdFactorPtrMap _factors;     /*!< Id to factor shared pointer container */
    IdVariablePtrMap _variables; /*!< Id to variable shared pointer container */

    void addVariable(VariableBase* v) override;
    void addFactor(FactorBase* f) override;

  private:
    Id _last_graph_id = 0; /*!< Last graph id */
    int _level_serialization = 0;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using FactorGraphPtr = std::shared_ptr<FactorGraph>; /*!< Shared pointer to FactorGraph */
} // namespace srrg2_solver
