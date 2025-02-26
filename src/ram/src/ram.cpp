#include "ram/ram.h"
#include "db_sta/dbNetwork.hh"
#include "layout.h"
#include "sta/FuncExpr.hh"
#include "sta/Liberty.hh"
#include "sta/PortDirection.hh"
#include "utl/Logger.h"
#include <cmath>  // For std::ceil and std::log2
#include <functional>
#include <limits>
#include <memory>
#include <vector>
#include <array>

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
      block_(nullptr),
      network_(nullptr),
      logger_(nullptr),
      storage_cell_(nullptr),
      tristate_cell_(nullptr),
      inv_cell_(nullptr),
      and2_cell_(nullptr),
      clock_gate_cell_(nullptr),
      gate_counter_(0),
      net_counter_(0),
      max_and_inputs_(2) {}

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
      logger_->error(RAM, 9, "term {} of cell {} not found.", mterm_name, master->getName());
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

// Generic buffer creation (NO ROTATION)
std::unique_ptr<Element> RamGen::make_buffer(
    const std::string& prefix,
    const std::vector<odb::dbNet*>& input_nets,
    const std::vector<odb::dbNet*>& output_nets,
    odb::Orientation2D orientation) { // Add orientation argument

  auto layout = std::make_unique<Layout>(orientation); // Use the provided orientation

  if (input_nets.size() != output_nets.size()) {
      logger_->error(RAM, 100, "Input and output net count mismatch in make_buffer");
      return nullptr; // Or throw an exception.
  }

  for (size_t bit = 0; bit < input_nets.size(); ++bit) {
    auto mid_net = makeNet(prefix, fmt::format("buffer_mid_{}", bit));

    // First inverter
    makeInst(layout.get(),
             prefix,
             fmt::format("inv1_{}", bit),
             inv_cell_,
             {{"A", input_nets[bit]}, {"Y", mid_net}});
    // NO ROTATION

    // Second inverter
    makeInst(layout.get(),
             prefix,
             fmt::format("inv2_{}", bit),
             inv_cell_,
             {{"A", mid_net}, {"Y", output_nets[bit]}});
    // NO ROTATION
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
    logger_->info(RAM, 6, "Generating {}", ram_name);

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

    // --- Layout Structure ---
    auto top_layout = std::make_unique<Layout>(odb::vertical);      // Top-level: Vertical
    auto core_layout = std::make_unique<Layout>(odb::horizontal); // Decoder + RAM + Addr Buffer: Horizontal
    auto decoder_layout = std::make_unique<Layout>(odb::vertical); // Decoder remains vertical


    // --- Create Ports (BTerms) ---
    auto clock = makeBTerm("clock");

    vector<dbNet*> write_enable(bytes_per_word, nullptr);
    for (int byte = 0; byte < bytes_per_word; ++byte) {
        write_enable[byte] = makeBTerm(fmt::format("write_enable[{}]", byte));
    }

    // --- Address Buffer ---
    int address_bits = static_cast<int>(std::ceil(std::log2(word_count)));
    std::vector<dbNet*> address_inputs_external(address_bits);
    for (int i = 0; i < address_bits; ++i) {
        address_inputs_external[i] = makeBTerm(fmt::format("A[{}]", i)); // External address inputs
    }

    std::vector<dbNet*> address_inputs_internal(address_bits);
    for (int i = 0; i < address_bits; ++i) {
        address_inputs_internal[i] = makeNet("ram", fmt::format("addr[{}]", i)); // Internal address nets
    }
    // Vertical layout for inverters *within* the address buffer
    auto address_buffer_layout = make_buffer("address_buffer", address_inputs_external, address_inputs_internal, odb::vertical);
    core_layout->addElement(std::move(address_buffer_layout)); // Add address buffer to core_layout

    // --- Decoder ---
    //Address buffer output is given as input to decoder.
    std::vector<dbNet*> decoder_outputs = buildDecoder(*decoder_layout, address_inputs_internal, word_count);
    core_layout->addElement(std::make_unique<Element>(std::move(decoder_layout))); // Add decoder to core_layout


    // --- Input Buffer (Data) ---
    std::vector<dbNet*> data_input_external(bits_per_word);
    for (int bit = 0; bit < bits_per_word; ++bit) {
        data_input_external[bit] = makeBTerm(fmt::format("Din[{}]", bit));
    }

    std::vector<dbNet*> data_input_internal(bits_per_word);
    for (int bit = 0; bit < bits_per_word; ++bit) {
        data_input_internal[bit] = makeNet("ram", fmt::format("Di0[{}]", bit));
    }
    auto input_buffer_layout = make_buffer("input_buffer", data_input_external, data_input_internal, odb::horizontal);
    top_layout->addElement(std::move(input_buffer_layout)); // Input buffer (Data) at the top level

    // --- Declare Do_internal OUTSIDE the column loop ---
    vector<array<dbNet*, 8>> Do_internal(read_ports);
    for (int read_port = 0; read_port < read_ports; ++read_port) {
        Do_internal[read_port].fill(nullptr); // Initialize the array
    }

    // --- RAM Columns ---
    for (int col = 0; col < bytes_per_word; ++col) {
        array<dbNet*, 8> Di0;
        for (int bit = 0; bit < 8; ++bit) {
        Di0[bit] = data_input_internal[bit + col * 8]; // Connect to input buffer
        }

        // Create INTERNAL Do nets
        for (int read_port = 0; read_port < read_ports; ++read_port) {
        for (int bit = 0; bit < 8; ++bit) {
            Do_internal[read_port][bit]
                = makeNet("ram", fmt::format("Do_internal_{}_{}", read_port, bit + col * 8));
        }
        }

        auto column_layout = std::make_unique<Layout>(odb::vertical);
        for (int row = 0; row < word_count; ++row) {
        column_layout->addElement(
            make_byte(fmt::format("storage_{}_{}", row, col),
                        read_ports,
                        clock,
                        write_enable[col],
                        {decoder_outputs[row]},
                        Di0,
                        Do_internal,
                        mask));
        }
        core_layout->addElement(std::make_unique<Element>(std::move(column_layout))); // Add RAM column
    }

    // --- Output Buffer ---
    // Create EXTERNAL Dout BTerms
    std::vector<dbNet*> data_output_external(bits_per_word * read_ports);
    int dout_index = 0;
    for(int read_port = 0; read_port < read_ports; ++read_port) {
        for(int bit = 0; bit < bits_per_word; ++bit)
        {
            data_output_external[dout_index] = makeBTerm(fmt::format("Dout[{}]", dout_index)); //fixed
            dout_index++; // Corrected increment
        }
    }

    // Flatten Do_internal for make_buffer - Now Do_internal is accessible!
    std::vector<dbNet*> data_output_internal_flat;
    for (int read_port = 0; read_port < read_ports; ++read_port)
    {
        for (int bit = 0; bit<bits_per_word; ++bit)
        {
            data_output_internal_flat.push_back(Do_internal[read_port][bit]);
        }
    }
    auto output_buffer_layout = make_buffer("output_buffer", data_output_internal_flat, data_output_external, odb::horizontal);

    top_layout->addElement(std::make_unique<Element>(std::move(core_layout)));
    top_layout->addElement(std::move(output_buffer_layout));

    // --- Final Layout Positioning ---
    top_layout->position(odb::Point(0, 0));
}

std::vector<dbNet*> RamGen::buildDecoder(Layout& parent_layout, const std::vector<dbNet*>& address_nets, int word_count)
{
  int address_bits = static_cast<int>(address_nets.size());
  // Create a top-level horizontal layout for the decoder
  auto decoder_layout = std::make_unique<Layout>(odb::horizontal);

  // Create a layout for all inverters
  auto inv_layout = std::make_unique<Layout>(odb::vertical);


  if (word_count == 4 && address_bits == 2) {
    auto and_layout = std::make_unique<Layout>(odb::vertical);
    // 2-to-4 decoder
    auto A0 = address_nets[0];
    auto A1 = address_nets[1];

    // Inverted signals
    auto A0_b = makeNet("decoder", "A0_b");
    auto A1_b = makeNet("decoder", "A1_b");

    // Place the inverters in the inv_layout
    makeInst(inv_layout.get(), "decoder", "invA0", inv_cell_, {{"A", A0}, {"Y", A0_b}});
    makeInst(inv_layout.get(), "decoder", "invA1", inv_cell_, {{"A", A1}, {"Y", A1_b}});

    std::vector<dbNet*> outputs(4);
    outputs[0] = makeNet("decoder", "word_select[0]");
    outputs[1] = makeNet("decoder", "word_select[1]");
    outputs[2] = makeNet("decoder", "word_select[2]");
    outputs[3] = makeNet("decoder", "word_select[3]");

    // Place the AND gates in the and_layout
    makeInst(and_layout.get(), "decoder", "and0", and2_cell_, {{"A", A1_b}, {"B", A0_b}, {"X", outputs[0]}});
    makeInst(and_layout.get(), "decoder", "and1", and2_cell_, {{"A", A1_b}, {"B", A0},   {"X", outputs[1]}});
    makeInst(and_layout.get(), "decoder", "and2", and2_cell_, {{"A", A1},   {"B", A0_b}, {"X", outputs[2]}});
    makeInst(and_layout.get(), "decoder", "and3", and2_cell_, {{"A", A1},   {"B", A0},   {"X", outputs[3]}});

    // Add the inv_layout and and_layout to the decoder_layout
    decoder_layout->addElement(std::make_unique<Element>(std::move(inv_layout)));
    decoder_layout->addElement(std::make_unique<Element>(std::move(and_layout)));

    // Add the completed decoder_layout to the parent_layout
    parent_layout.addElement(std::make_unique<Element>(std::move(decoder_layout)));

    return outputs;

  }else if (word_count == 8 && address_bits == 3) {
	  // 3-to-8 decoder
          auto and_layout = std::make_unique<Layout>(odb::horizontal);
	  auto A0 = address_nets[0];
	  auto A1 = address_nets[1];
	  auto A2 = address_nets[2];

	  // Inverted signals
	  auto A0_b = makeNet("decoder", "A0_b");
	  auto A1_b = makeNet("decoder", "A1_b");
	  auto A2_b = makeNet("decoder", "A2_b");

	  // Place inverters in inv_layout
	  makeInst(inv_layout.get(), "decoder", "invA0", inv_cell_,
		   {{"A", A0}, {"Y", A0_b}});
	  makeInst(inv_layout.get(), "decoder", "invA1", inv_cell_,
		   {{"A", A1}, {"Y", A1_b}});
	  makeInst(inv_layout.get(), "decoder", "invA2", inv_cell_,
		   {{"A", A2}, {"Y", A2_b}});

	  // Prepare output nets
	  std::vector<dbNet*> outputs(8);
	  for (int i = 0; i < 8; i++) {
	    outputs[i] = makeNet("decoder", fmt::format("word_select[{}]", i));
	  }

	  // Helper lambda for a 3-input AND via two 2-input ANDs.
	  auto triple_and = [&](Layout* layout_ptr,
		                const std::string& prefix,
		                dbNet* in1, dbNet* in2, dbNet* in3,
		                dbNet* out)
	  {
	    auto mid_net = makeNet("decoder", fmt::format("{}_mid", prefix));
	    makeInst(layout_ptr, "decoder", fmt::format("{}_and1", prefix),
		     and2_cell_, {{"A", in1}, {"B", in2}, {"X", mid_net}});
	    makeInst(layout_ptr, "decoder", fmt::format("{}_and2", prefix),
		     and2_cell_, {{"A", mid_net}, {"B", in3}, {"X", out}});
	  };

	  // Create two horizontal layouts for the AND gates (top and bottom rows)
	  auto and_layout_top = std::make_unique<Layout>(odb::vertical);
	  auto and_layout_bottom = std::make_unique<Layout>(odb::vertical);

	  // Top row: out0..out3
	  triple_and(and_layout_top.get(), "out0", A2_b, A1_b, A0_b, outputs[0]);
	  triple_and(and_layout_top.get(), "out1", A2_b, A1_b, A0,   outputs[1]);
	  triple_and(and_layout_top.get(), "out2", A2_b, A1,   A0_b, outputs[2]);
	  triple_and(and_layout_top.get(), "out3", A2_b, A1,   A0,   outputs[3]);

	  // Bottom row: out4..out7
	  triple_and(and_layout_bottom.get(), "out4", A2, A1_b, A0_b, outputs[4]);
	  triple_and(and_layout_bottom.get(), "out5", A2, A1_b, A0,   outputs[5]);
	  triple_and(and_layout_bottom.get(), "out6", A2, A1,   A0_b, outputs[6]);
	  triple_and(and_layout_bottom.get(), "out7", A2, A1,   A0,   outputs[7]);

	  // Add the two horizontal rows to the vertical 'and_layout'
	  and_layout->addElement(std::make_unique<Element>(std::move(and_layout_top)));
	  and_layout->addElement(std::make_unique<Element>(std::move(and_layout_bottom)));

	  // Add the inv_layout and and_layout to the decoder_layout (horizontally)
	  decoder_layout->addElement(std::make_unique<Element>(std::move(inv_layout)));
	  decoder_layout->addElement(std::make_unique<Element>(std::move(and_layout)));

	  // Add the completed decoder_layout to the parent_layout
	  parent_layout.addElement(std::make_unique<Element>(std::move(decoder_layout)));

	  return outputs;
	}else {
    logger_->error(RAM, 12, "Unsupported decoder configuration (word_count = {}). Only 4 or 8 words are supported.", word_count);
    return {};
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
          if (!port->direction()->isOutput()) {
            return false;
          }
          auto function = port->function();
          if (!function) {
            return false;
          }
          return function->op() == sta::FuncExpr::op_port && port->direction()->isTristate();
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

  max_and_inputs_ = 2;
  and_cells_[2] = and2_cell_;

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
  bool is_and_gate = isAndGateFunction(function, inputs_count);

  return is_and_gate && inputs_count == num_inputs;
}

bool RamGen::isAndGateFunction(sta::FuncExpr* expr, int& inputs_count) {
  if (expr->op() == sta::FuncExpr::op_port) {
    inputs_count += 1;
    return true;
  } else if (expr->op() == sta::FuncExpr::op_and) {
    bool left_is_and = isAndGateFunction(expr->left(), inputs_count);
    bool right_is_and = isAndGateFunction(expr->right(), inputs_count);
    return left_is_and && right_is_and;
  } else {
    return false;
  }
}

odb::dbMaster* RamGen::getAndGate(int num_inputs) {
  return and2_cell_;
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
