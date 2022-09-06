using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MGroup.MSolve.Discretization.Entities;

namespace ConvectionDiffusionTest
{
    public class ComsolMeshReader
    {
        private enum ReadingStatus
        {
            Unknown,
            FoundNodes,
            ReadingNodes,
            FoundHexa,
            ReadingHexa
        }

        public Dictionary<int, Node> NodesDictionary { get; internal set; }
        public Dictionary<int, Node[]> ElementConnectivity { get; internal set; }
        
        public ComsolMeshReader(string filepath)
        {
            NodesDictionary = new Dictionary<int, Node>();
            ElementConnectivity = new Dictionary<int, Node[]>();
            
            try
            {
                using (var sr = new StreamReader(filepath))
                {
                    ReadingStatus status = ReadingStatus.Unknown;
                    Console.WriteLine("Reading file...");
                    var line = sr.ReadLine();
                    var id = 0;
                    while (line != null)
                    {
                        //Status update
                        if (line.Equals("# Mesh vertex coordinates"))
                        {
                            status = ReadingStatus.FoundNodes;
                            Console.WriteLine("Status: Found nodes");
                        }
                        else if (status == ReadingStatus.FoundNodes)
                        {
                            status = ReadingStatus.ReadingNodes;
                            Console.WriteLine("Status: Reading nodes");
                            id = 0;
                        }
                        else if (line.Equals("3 hex # type name"))
                        {
                            status = ReadingStatus.FoundHexa;
                            Console.WriteLine("Status: Found Hexa");
                        }
                        else if(status == ReadingStatus.FoundHexa && !line.Contains("#") && !line.Equals(""))
                        {
                            status = ReadingStatus.ReadingHexa;
                            Console.WriteLine("Status: Reading Hexa");
                            id = 0;
                        }
                        else if (line.Equals("") && status != ReadingStatus.Unknown && status != ReadingStatus.FoundHexa)
                        {
                            status = ReadingStatus.Unknown;
                            Console.WriteLine("Status: Unknown");
                        }

                        //Action
                        if (status == ReadingStatus.ReadingNodes)
                        {//Nodes
                            //Split line
                            var coordsString = line.Split(" ");
                            //Convert to double
                            var coords = new double[coordsString.GetLength(0) - 1];
                            for (int i = 0; i < coords.Length; i++)
                                coords[i] = double.Parse(coordsString[i]);

                            NodesDictionary.Add(key: id, new Node(id: id, x: coords[0], y: coords[1], z: coords[2]));

                            //Print
                            string identation = id < 10 ? " " : "";
                            Console.WriteLine("Node {0}{1}: ({2}, {3}, {4})", id, identation, coords[0].ToString("F2"), coords[1].ToString("F2"), coords[2].ToString("F2"));
                            //Increment id
                            id++;
                        }
                        else if (status == ReadingStatus.ReadingHexa)
                        {
                            //Split line
                            var nodesString = line.Split(" ");
                            //Convert to int
                            var nodeIDs = new int[nodesString.GetLength(0) - 1];
                            for (int i = 0; i < nodeIDs.Length; i++)
                                nodeIDs[i] = int.Parse(nodesString[i]);
                            //Identify nodes and reorder to match MSolve convention
                            var nodes = new Node[8];
                            var reorderedNodes = new int[] { 6, 7, 5, 4, 2, 3, 1, 0 };
                            for (int i = 0; i < 8; i++)
                                nodes[reorderedNodes[i]] = NodesDictionary[nodeIDs[i]];

                            ElementConnectivity.Add(key: id, value: nodes);

                            //Print
                            Console.WriteLine("Element {0}", id);
                            for (int i = 0; i < nodeIDs.Length; i++)
                            {
                                string identation = nodeIDs[i] < 10 ? " " : "";
                                Console.WriteLine("\tNode {0}{1}: ({2}, {3}, {4})", nodeIDs[i], identation, NodesDictionary[nodeIDs[i]].X.ToString("F2"), NodesDictionary[nodeIDs[i]].Y.ToString("F2"), NodesDictionary[nodeIDs[i]].Z.ToString("F2"));
                            }
                            //Increment id
                            id++;
                        }

                        //Read next line
                        line = sr.ReadLine();
                    }
                }
            }
            catch (IOException e)
            {
                Console.WriteLine("The file could not be read:");
                Console.WriteLine(e.Message);
            }
            Console.WriteLine("Finished reading file");
        }
    }
}
