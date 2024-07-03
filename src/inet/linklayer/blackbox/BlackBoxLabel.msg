//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

//
// TODO generated message class
//

import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;
import inet.common.Units;
import inet.networklayer.common.L3Address;

namespace inet::blackbox;

cplusplus {{
     typedef  std::deque<unsigned int>  LabelList;
     typedef  std::deque<MacAddress>  NodesList;
      }} // C++ typedef
      
class LabelList { @existingClass; }
class NodesList { @existingClass; }

class BlackBoxLabel extends FieldsChunk {
    LabelList labels;
    NodesList nodesVirtual;
    NodesList nodesReal;    
}

