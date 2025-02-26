#pragma once

#include <functional>
#include <memory>
#include <map>  // For storing different AND gate masters
#include <vector>
#include <array>
#include "odb/db.h"
#include "sta/FuncExpr.hh"  // For sta::FuncExpr
#include "sta/Liberty.hh"   // For sta::LibertyPort and sta::LibertyCell
#include "sta/PortDirection.hh"  // For sta::PortDirection
#include "utl/Logger.h"
#include "../../src/layout.h" // Include the layout header

namespace odb {
class dbMaster;
class dbNet;  // Forward declaration of dbNet
}

namespace sta {
class dbNetwork;
class LibertyPort;
class LibertyCell;
class PortDirection;
}  // namespace sta

namespace ram {

using utl::Logger;

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
      const std::vector<odb::dbNet*>& select_signals,
      const std::array<odb::dbNet*, 8>& data_input,
      const std::vector<std::array<odb::dbNet*, 8>>& data_output,
      bool mask);

  // Generic buffer creation
    std::unique_ptr<Element> make_buffer(
      const std::string& prefix,
      const std::vector<odb::dbNet*>& input_nets,
      const std::vector<odb::dbNet*>& output_nets,
      odb::Orientation2D orientation); // Add orientation


  // Decoder builder for 2-to-4 and 3-to-8 decoders
  std::vector<odb::dbNet*> buildDecoder(Layout& layout, const std::vector<odb::dbNet*>& address_nets, int word_count);

  odb::dbMaster* getAndGate(int num_inputs);
  bool isAndGate(sta::LibertyPort* port, int num_inputs);
  bool isAndGateFunction(sta::FuncExpr* expr, int& inputs_count);

  void findMasters();
  odb::dbMaster* findMaster(const std::function<bool(sta::LibertyPort*)>& match,
                            const char* name);

  odb::dbDatabase* db_;
  odb::dbBlock* block_;
  sta::dbNetwork* network_;
  Logger* logger_;

  odb::dbMaster* storage_cell_;
  odb::dbMaster* tristate_cell_;
  odb::dbMaster* inv_cell_;
  odb::dbMaster* and2_cell_;
  odb::dbMaster* clock_gate_cell_;

  int gate_counter_;   // Counter for unique gate instances
  int net_counter_;    // Counter for unique net instances
  int max_and_inputs_; // Maximum number of inputs an AND gate can have
  std::map<int, odb::dbMaster*> and_cells_; // Map to store AND gate masters by number of inputs
};

}  // namespace ram
