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

#include "LMPR.h"

namespace inet {

void LMPR::handleDataFromUpperLayer(Packet *packet)
{
    auto lh = makeShared<LMPRHeader>();

    auto l3AddressReq = packet->removeTag<L3AddressReq>();
    Ipv4Address dest = l3AddressReq->getDestAddress().toIpv4();

    lh->setSrcAddr(m_selfIpv4Address);
    lh->setDestAddr(dest);
    lh->setSeqNum(m_seqNum);
    m_seqNum++;
    lh->setTtl(defaultTTL);
    int totalByteLength = sizeof(m_selfIpv4Address)*2 + sizeof((int)defaultTTL)*3;
    lh->setChunkLength(B(totalByteLength));

    packet->insertAtFront(lh);

    forwardData(packet, dest);
}


void LMPR::handleDataFromLowerLayer(Packet *packet)
{
    auto lh = packet->peekAtFront<LMPRHeader>();
    // forward
    Ipv4Address src = lh->getSourceAddress().toIpv4();
    Ipv4Address dest = lh->getDestinationAddress().toIpv4();
    int ttl = lh->getTtl();
    if(dest == m_selfIpv4Address)
    {
        lh = packet->popAtFront<LMPRHeader>();
        packet->addTagIfAbsent<NetworkProtocolInd>()->setProtocol(&getProtocol());
        packet->addTagIfAbsent<NetworkProtocolInd>()->setNetworkProtocolHeader(lh);
        packet->addTagIfAbsent<DispatchProtocolReq>()->setProtocol(&Protocol::udp);
        packet->addTagIfAbsent<PacketProtocolTag>()->setProtocol(&Protocol::udp);
        packet->addTagIfAbsent<HopLimitInd>()->setHopLimit(ttl);
        auto l3AddressInd = packet->addTagIfAbsent<L3AddressInd>();
        l3AddressInd->setSrcAddress(lh->getSourceAddress());
        l3AddressInd->setDestAddress(lh->getDestinationAddress());
        sendUp(packet);
    }
    else       // forward
    {
        forwardData(packet, dest);
    }
}

void LMPR::forwardData(Packet* packet, Ipv4Address dest)
{
    forwardData_Optimal(packet, dest);
//    forwardData_Forecast(packet, dest);
}

void LMPR::forwardData_Optimal(Packet* packet, Ipv4Address dest)
{
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (i != j) {  // No self-loops
                Coord PosA = _globalMob[i].second->getCurrentPosition();
                Coord PosB = _globalMob[j].second->getCurrentPosition();
                double distance = PosA.distance(PosB);
                double commRange = getRealCommRange(PosA, PosB);
                if (distance <= commRange) {
                    adjacencyMatrix[i][j] = 1.0; // Nodes i and j are within communication range
                } else {
                    adjacencyMatrix[i][j] = 0.0;
                }
            }
        }
    }
    int nextNodeIdx = dijkstra(adjacencyMatrix, m_selfNodeIdx, getTargetNodeIdxByAddr(dest));
    Ipv4Address nextHopAddr = _globalMob[nextNodeIdx].first;
    MacAddress nextHopMacAddr = arp->resolveL3Address(nextHopAddr, nullptr);
    setDownControlInfo(packet, nextHopMacAddr);
    sendDown(packet);
}

void LMPR::forwardData_Forecast(Packet* packet, Ipv4Address dest)
{
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (i != j) {  // No self-loops
                NodeInfo* node_info_A = getNodeInfoByAddr(_globalMob[i].first);
                NodeInfo* node_info_B = getNodeInfoByAddr(_globalMob[j].first);
                if(node_info_A && node_info_B)
                {
                    Coord PosA = forecastNodeLocation(node_info_A, simTime());
                    Coord PosB = forecastNodeLocation(node_info_B, simTime());
                    double distance = PosA.distance(PosB);
                    double commRange = getRealCommRange(PosA, PosB);
                    if (distance <= commRange) {
                        adjacencyMatrix[i][j] = 1.0; // Nodes i and j are within communication range
                    } else {
                        adjacencyMatrix[i][j] = 0.0;
                    }
                }
                else
                {
                    adjacencyMatrix[i][j] = 0.0;
                }
            }
        }
    }
}

int LMPR::dijkstra(const vector<vector<double>>& graph, int source, int target) {
    int N = graph.size();
    vector<double> dist(N, INF);  // Distance from source to each node
    vector<int> prev(N, -1);   // Previous node in optimal path
    vector<bool> visited(N, false);  // Whether the node has been processed

    // Priority queue to select the node with the smallest distance
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    // Initialize the source node
    dist[source] = 0;
    pq.push({0, source});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (visited[u]) continue;
        visited[u] = true;

        // Check neighbors of u
        for (int v = 0; v < N; ++v) {
            if (graph[u][v] != 0 && !visited[v]) {  // There's an edge between u and v
                int newDist = dist[u] + graph[u][v];
                if (newDist < dist[v]) {
                    dist[v] = newDist;
                    prev[v] = u;
                    pq.push({dist[v], v});
                }
            }
        }
    }

    // Output the shortest path
    if (dist[target] == INF) {
        return -1;
    } else {
        vector<int> path;
        for (int v = target; v != -1; v = prev[v]) {
            path.push_back(v);
        }
        reverse(path.begin(), path.end());
        return path[1];
    }
}


} /* namespace inet */
