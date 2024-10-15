/////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2023, Precision Innovations Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////


#include "ram/ram.h"
#include "db_sta/dbNetwork.hh"
#include "layout.h"
#include "sta/FuncExpr.hh"
#include "sta/Liberty.hh"
#include "sta/PortDirection.hh"
#include "utl/Logger.h"

#include <cmath>  // For std::ceil and std::log2

namespace ram {

using odb::dbBlock;
using odb::dbBTerm;
using odb::dbInst;
using odb::dbMaster;
using odb::dbNet;
using utl::RAM;
using std::vector;
using std::array;

RamGen::RamGen()
    : db_(nullptr),
      logger_(nullptr),
      gate_counter_(0),
      net_counter_(0),
      max_and_inputs_(4) {}  // Initialize counters and max AND inputs

void RamGen::init(odb::dbDatabase* db, sta::dbNetwork* network, Logger* logger) {
  db_ = db;
  network_ = network;
  logger_ = logger;
}

dbInst* RamGen::makeInst(
    Layout* layout,
    const std::string& prefix,
    const std::string& name,
    dbMaster* master,
    const vector<std::pair<std::string, dbNet*>>& connections) {
  const auto inst_name = fmt::format("{}.{}", prefix, name);
  auto inst = dbInst::create(block_, master, inst_name.c_str());
  for (auto& [mterm_name, net] : connections) {
    auto mterm = master->findMTerm(mterm_name.c_str());
    if (!mterm) {
      logger_->error(RAM, 9, "term {} of cell {} not found.", name, master->getName());
    }
    auto iterm = inst->getITerm(mterm);
    iterm->connect(net);
  }

  layout->addElement(std::make_unique<Element>(inst));
  return inst;
}

dbNet* RamGen::makeNet(const std::string& prefix, const std::string& name) {
  const auto net_name = fmt::format("{}.{}", prefix, name);
  return dbNet::create(block_, net_name.c_str());
}

dbNet* RamGen::makeBTerm(const std::string& name) {
  auto net = dbNet::create(block_, name.c_str());
  dbBTerm::create(net, name.c_str());
  return net;
}

std::unique_ptr<Element> RamGen::make_bit(const std::string& prefix,
                                          const int read_ports,
                                          dbNet* clock,
                                          vector<odb::dbNet*>& select,
                                          dbNet* data_input,
                                          vector<odb::dbNet*>& data_output) {
  auto layout = std::make_unique<Layout>(odb::horizontal);
  auto storage_net = makeNet(prefix, "storage");

  // Make Storage latch
  makeInst(layout.get(),
           prefix,
           "bit",
           storage_cell_,
           {{"GATE", clock}, {"D", data_input}, {"Q", storage_net}});

  // Make output tristate driver(s) for read port(s)
  for (int read_port = 0; read_port < read_ports; ++read_port) {
    makeInst(layout.get(),
             prefix,
             fmt::format("obuf{}", read_port),
             tristate_cell_,
             {{"A", storage_net}, {"TE_B", select[read_port]}, {"Z", data_output[read_port]}});
  }

  return std::make_unique<Element>(std::move(layout));
}

std::unique_ptr<Element> RamGen::make_byte(
    const std::string& prefix,
    const int read_ports,
    dbNet* clock,
    dbNet* write_enable,
    const std::vector<dbNet*>& select_signals,
    const std::array<dbNet*, 8>& data_input,
    const std::vector<std::array<dbNet*, 8>>& data_output,
    bool mask) {
  auto layout = std::make_unique<Layout>(odb::horizontal);

  // Use the select signal from the decoder
  dbNet* select = select_signals[0];

  // Combine select with write_enable using an AND gate to create we_net
  dbNet* we_net = makeNet(prefix, "we_net");
  makeInst(layout.get(),
           prefix,
           "we_and",
           and2_cell_,
           {{"A", select}, {"B", write_enable}, {"X", we_net}});

  dbNet* clock_b_net = makeNet(prefix, "clock_b");
  dbNet* gclock_net = makeNet(prefix, "gclock");

  // Invert clock
  makeInst(layout.get(),
           prefix,
           "clock_inv",
           inv_cell_,
           {{"A", clock}, {"Y", clock_b_net}});

  // Make clock gate
  makeInst(layout.get(),
           prefix,
           "icg",
           clock_gate_cell_,
           {{"CLK", clock_b_net}, {"GATE", we_net}, {"GCLK", gclock_net}});

  // Create select_b (inverted select) for read ports
  vector<dbNet*> select_b_nets(read_ports);
  for (int i = 0; i < read_ports; ++i) {
    select_b_nets[i] = makeNet(prefix, fmt::format("select{}_b", i));
    makeInst(layout.get(),
             prefix,
             fmt::format("select_inv_{}", i),
             inv_cell_,
             {{"A", select}, {"Y", select_b_nets[i]}});
  }

  // Create bits
  for (int bit = 0; bit < 8; ++bit) {
    auto name = fmt::format("{}.bit{}", prefix, bit);
    vector<dbNet*> outs;
    for (int read_port = 0; read_port < read_ports; ++read_port) {
      outs.push_back(data_output[read_port][bit]);
    }
    layout->addElement(make_bit(name, read_ports, gclock_net, select_b_nets, data_input[bit], outs));
  }

  return std::make_unique<Element>(std::move(layout));
}

void RamGen::generate(const int bytes_per_word,
                      const int word_count,
                      const int read_ports,
                      dbMaster* storage_cell,
                      dbMaster* tristate_cell,
                      dbMaster* inv_cell,
                      bool mask) {
  const int bits_per_word = bytes_per_word * 8;
  const std::string ram_name = fmt::format("RAM{}x{}", word_count, bits_per_word);

  logger_->info(RAM, 3, "Generating {}", ram_name);

  storage_cell_ = storage_cell;
  tristate_cell_ = tristate_cell;
  inv_cell_ = inv_cell;
  and2_cell_ = nullptr;
  clock_gate_cell_ = nullptr;
  findMasters();

  auto chip = db_->getChip();
  if (!chip) {
    chip = odb::dbChip::create(db_);
  }

  block_ = chip->getBlock();
  if (!block_) {
    block_ = odb::dbBlock::create(chip, ram_name.c_str());
  }

  Layout layout(odb::horizontal);

  auto clock = makeBTerm("clock");

  vector<dbNet*> write_enable(bytes_per_word, nullptr);
  for (int byte = 0; byte < bytes_per_word; ++byte) {
    auto in_name = fmt::format("write_enable[{}]", byte);
    write_enable[byte] = makeBTerm(in_name);
  }

  // Calculate address bits
  int address_bits = static_cast<int>(std::ceil(std::log2(word_count)));
  std::vector<dbNet*> address_nets(address_bits);
  for (int i = 0; i < address_bits; ++i) {
    auto addr_name = fmt::format("addr[{}]", i);
    address_nets[i] = makeBTerm(addr_name);
  }

  // Create inverted address nets
  std::vector<dbNet*> addr_inverted(address_bits);
  for (int bit = 0; bit < address_bits; ++bit) {
    auto inv_net_name = fmt::format("addr{}_b", bit);
    addr_inverted[bit] = makeNet("decoder", inv_net_name);
    makeInst(
        &layout,
        "decoder",
        fmt::format("inv_addr{}", bit),
        inv_cell_,
        {{"A", address_nets[bit]}, {"Y", addr_inverted[bit]}});
  }

  // Create decoder outputs
  std::vector<dbNet*> decoder_outputs(word_count);
  for (int i = 0; i < word_count; ++i) {
    auto select_name = fmt::format("word_select[{}]", i);
    decoder_outputs[i] = makeNet("decoder", select_name);
  }

  // Build decoder logic
  for (int i = 0; i < word_count; ++i) {
    std::vector<dbNet*> input_signals;
    for (int bit = 0; bit < address_bits; ++bit) {
      bool bit_value = (i >> bit) & 1;
      if (bit_value) {
        input_signals.push_back(address_nets[bit]);
      } else {
        input_signals.push_back(addr_inverted[bit]);
      }
    }

    // Create decoder logic
    createDecoderLogic(layout, decoder_outputs[i], input_signals);
  }

  // For each byte in the word
  for (int col = 0; col < bytes_per_word; ++col) {
    array<dbNet*, 8> Di0;
    for (int bit = 0; bit < 8; ++bit) {
      Di0[bit] = makeBTerm(fmt::format("Di0[{}]", bit + col * 8));
    }

    vector<array<dbNet*, 8>> Do;
    for (int read_port = 0; read_port < read_ports; ++read_port) {
      array<dbNet*, 8> d;
      for (int bit = 0; bit < 8; ++bit) {
        auto out_name = fmt::format("Do{}[{}]", read_port, bit + col * 8);
        d[bit] = makeBTerm(out_name);
      }
      Do.push_back(d);
    }

    auto column = std::make_unique<Layout>(odb::vertical);
    for (int row = 0; row < word_count; ++row) {
      auto name = fmt::format("storage_{}_{}", row, col);
      column->addElement(make_byte(name,
                                   read_ports,
                                   clock,
                                   write_enable[col],
                                   {decoder_outputs[row]},  // Use decoder output
                                   Di0,
                                   Do,
                                   mask));
    }
    layout.addElement(std::make_unique<Element>(std::move(column)));
  }
  layout.position(odb::Point(0, 0));
}

void RamGen::createDecoderLogic(Layout& layout, dbNet* output_net, const std::vector<dbNet*>& input_nets) {
  if (input_nets.size() <= max_and_inputs_) {
    // Create an AND gate with available inputs
    auto and_gate = getAndGate(input_nets.size());
    if (!and_gate) {
      logger_->error(RAM, 12, "No AND gate found for {} inputs.", input_nets.size());
    }
    std::vector<std::pair<std::string, dbNet*>> connections;
    for (int j = 0; j < input_nets.size(); ++j) {
      connections.push_back({fmt::format("A{}", j), input_nets[j]});
    }
    connections.push_back({"X", output_net});
    makeInst(
        &layout,
        "decoder",
        fmt::format("and_gate_{}", gate_counter_++),
        and_gate,
        connections);
  } else {
    // Split the inputs and create intermediate nets
    int mid = input_nets.size() / 2;
    std::vector<dbNet*> left_inputs(input_nets.begin(), input_nets.begin() + mid);
    std::vector<dbNet*> right_inputs(input_nets.begin() + mid, input_nets.end());

    // Create intermediate nets
    dbNet* left_net = makeNet("decoder", fmt::format("intermediate_net_{}", net_counter_++));
    dbNet* right_net = makeNet("decoder", fmt::format("intermediate_net_{}", net_counter_++));

    // Recursively create logic for left and right halves
    createDecoderLogic(layout, left_net, left_inputs);
    createDecoderLogic(layout, right_net, right_inputs);

    // Combine left and right nets with an AND gate
    makeInst(
        &layout,
        "decoder",
        fmt::format("and_gate_{}", gate_counter_++),
        and2_cell_,
        {{"A", left_net}, {"B", right_net}, {"X", output_net}});
  }
}

void RamGen::findMasters() {
  if (!inv_cell_) {
    inv_cell_ = findMaster(
        [this](sta::LibertyPort* port) {
          return port->libertyCell()->isInverter();
        },
        "inverter");
  }

  if (!tristate_cell_) {
    tristate_cell_ = findMaster(
        [this](sta::LibertyPort* port) {
          if (!port->direction()->isTristate()) {
            return false;
          }
          auto function = port->function();
          return function && function->op() != sta::FuncExpr::op_not;
        },
        "tristate");
  }

  if (!and2_cell_) {
    and2_cell_ = findMaster(
        [this](sta::LibertyPort* port) {
          return isAndGate(port, 2);
        },
        "and2");
  }

  // Find AND gates with more inputs up to max_and_inputs_
  for (int i = 3; i <= max_and_inputs_; ++i) {
    and_cells_[i] = findMaster(
        [this, i](sta::LibertyPort* port) {
          return isAndGate(port, i);
        },
        fmt::format("and{}", i).c_str());
  }

  if (!storage_cell_) {
    storage_cell_ = findMaster(
        [this](sta::LibertyPort* port) {
          return port->libertyCell()->hasSequentials();
        },
        "storage");
  }

  if (!clock_gate_cell_) {
    clock_gate_cell_ = findMaster(
        [this](sta::LibertyPort* port) {
          return port->libertyCell()->isClockGate();
        },
        "clock gate");
  }
}

bool RamGen::isAndGate(sta::LibertyPort* port, int num_inputs) {
  if (!port->direction()->isOutput()) {
    return false;
  }
  auto function = port->function();
  if (!function) {
    return false;
  }
  int inputs_count = 0;
  // Recursively count the number of input ports in the function
  std::function<int(sta::FuncExpr*)> countInputs = [&](sta::FuncExpr* expr) -> int {
    if (expr->op() == sta::FuncExpr::op_port) {
      return 1;
    } else if (expr->op() == sta::FuncExpr::op_and) {
      return countInputs(expr->left()) + countInputs(expr->right());
    } else {
      return 0;
    }
  };
  inputs_count = countInputs(function);
  return (inputs_count == num_inputs);
}

odb::dbMaster* RamGen::getAndGate(int num_inputs) {
  if (num_inputs == 2) {
    return and2_cell_;
  } else if (and_cells_.find(num_inputs) != and_cells_.end()) {
    return and_cells_[num_inputs];
  } else {
    // For more inputs, chain smaller gates or handle error
    return nullptr;
  }
}

odb::dbMaster* RamGen::findMaster(
    const std::function<bool(sta::LibertyPort*)>& match,
    const char* name) {
  dbMaster* best = nullptr;
  float best_area = std::numeric_limits<float>::max();

  for (auto lib : db_->getLibs()) {
    for (auto master : lib->getMasters()) {
      auto cell = network_->dbToSta(master);
      if (!cell) {
        continue;
      }
      auto liberty = network_->libertyCell(cell);
      if (!liberty) {
        continue;
      }

      auto port_iter = liberty->portIterator();

      sta::ConcretePort* out = nullptr;
      while (port_iter->hasNext()) {
        auto lib_port = port_iter->next();
        auto dir = lib_port->direction();
        if (dir->isAnyOutput()) {
          if (!out) {
            out = lib_port;
          } else {
            out = nullptr;  // no multi-output gates
            break;
          }
        }
      }

      delete port_iter;
      if (!out || !match(out->libertyPort())) {
        continue;
      }

      if (liberty->area() < best_area) {
        best_area = liberty->area();
        best = master;
      }
    }
  }

  if (!best) {
    logger_->error(RAM, 10, "Can't find {} cell", name);
  }
  logger_->info(RAM, 16, "Selected {} cell {}", name, best->getName());
  return best;
}

}  // namespace ram

