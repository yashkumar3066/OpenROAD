#pragma once

#include <functional>
#include <memory>
#include <map>  // For storing different AND gate masters
#include "odb/db.h"
#include "sta/FuncExpr.hh"  // For sta::FuncExpr
#include "sta/Liberty.hh"  // For sta::LibertyPort and sta::LibertyCell
#include "sta/PortDirection.hh"  // For sta::PortDirection

namespace odb {
class dbMaster;
}

namespace sta {
class dbNetwork;
class LibertyPort;
class LibertyCell;
class PortDirection;
}  // namespace sta

namespace utl {
class Logger;
}

namespace ram {

using utl::Logger;

////////////////////////////////////////////////////////////////
class Element;
class Layout;

class RamGen
{
 public:
  RamGen();

  void init(odb::dbDatabase* db, sta::dbNetwork* network, Logger* logger);
  
  void generate(const int bytes_per_word,
                const int word_count,
                const int read_ports,
                odb::dbMaster* storage_cell,
                odb::dbMaster* tristate_cell,
                odb::dbMaster* inv_cell,
                bool mask);

  void findMasters();
  odb::dbMaster* findMaster(const std::function<bool(sta::LibertyPort*)>& match,
                            const char* name);

 private:
  odb::dbNet* makeNet(const std::string& prefix, const std::string& name);
  odb::dbInst* makeInst(
      Layout* layout,
      const std::string& prefix,
      const std::string& name,
      odb::dbMaster* master,
      const std::vector<std::pair<std::string, odb::dbNet*>>& connections);
  odb::dbNet* makeBTerm(const std::string& name);

  std::unique_ptr<Element> make_bit(const std::string& prefix,
                                    const int read_ports,
                                    odb::dbNet* clock,
                                    std::vector<odb::dbNet*>& select,
                                    odb::dbNet* data_input,
                                    std::vector<odb::dbNet*>& data_output);

  std::unique_ptr<Element> make_byte(
      const std::string& prefix,
      const int read_ports,
      odb::dbNet* clock,
      odb::dbNet* write_enable,
      const std::vector<odb::dbNet*>& select,
      const std::array<odb::dbNet*, 8>& data_input,
      const std::vector<std::array<odb::dbNet*, 8>>& data_output,
      bool mask);

  // New function declarations for decoder logic
  void createDecoderLogic(Layout& layout, odb::dbNet* output_net, const std::vector<odb::dbNet*>& input_nets);
  odb::dbMaster* getAndGate(int num_inputs);
  bool isAndGate(sta::LibertyPort* port, int num_inputs);

  // Member variables for the class
  odb::dbDatabase* db_;
  odb::dbBlock* block_;
  sta::dbNetwork* network_;
  Logger* logger_;

  odb::dbMaster* storage_cell_;
  odb::dbMaster* tristate_cell_;
  odb::dbMaster* inv_cell_;
  odb::dbMaster* and2_cell_;
  odb::dbMaster* clock_gate_cell_;

  // New member variables for gate and net management
  int gate_counter_;  // Counter for unique gate instances
  int net_counter_;   // Counter for unique net instances
  int max_and_inputs_;  // Maximum number of inputs an AND gate can have
  std::map<int, odb::dbMaster*> and_cells_;  // A map to store AND gate masters by the number of inputs
};

}  // namespace ram

