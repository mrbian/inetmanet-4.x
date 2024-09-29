import inet.common.INETDefs;
import inet.common.packet.chunk.Chunk;
import inet.networklayer.contract.ipv4.Ipv4Address;
namespace inet;

class OGM extends FieldsChunk 
{
    Ipv4Address origin;
    int seq;
    float x1;
    float y1;
    float x2;
    float y2;
    float x3;
    float y3;
//    double timeOffset;
	double timeAbstract;
    int TTL;
}