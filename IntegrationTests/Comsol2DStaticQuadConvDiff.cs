using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Isoparametric;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;

namespace ConvectionDiffusionTest
{
    public static class Comsol2DStaticQuadConvDiff
    {   
        public static double[] ConvectionCoeff => new[]  {1d, 1d};
        public static double DiffusionCoeff => 1d;
        
        //                                                     5                  6                 9                10
        private static double[] prescribedSolution = { 95.48802946592981, 83.2412523020255, 95.48802946592977, 83.24125230202547};
        // private static double[] prescribedSolution = { 83.33333333333333, 66.66666666666667, 83.33333333333333, 66.66666666666667};
        public static Model CreateModel()
        {
            var model = new Model();
            model.SubdomainsDictionary.Add(0, new Subdomain(0));
            var nodes = new Node[]
            {
                new Node(id : 1, x : 0, y : 0),
                new Node(id : 2, x : 1, y : 0),
                new Node(id : 3, x : 2, y : 0),
                new Node(id : 4, x : 3, y : 0),
                new Node(id : 5, x : 0, y : 1),
                new Node(id : 6, x : 1, y : 1),
                new Node(id : 7, x : 2, y : 1),
                new Node(id : 8, x : 3, y : 1),
                new Node(id : 9, x : 0, y : 2),
                new Node(id : 10, x : 1, y : 2),
                new Node(id : 11, x : 2, y : 2),
                new Node(id : 12, x : 3, y : 2),
                new Node(id : 13, x : 0, y : 3),
                new Node(id : 14, x : 1, y : 3),
                new Node(id : 15, x : 2, y : 3),
                new Node(id : 16, x : 3, y : 3),
            };
            foreach (var node in nodes)
            {
                model.NodesDictionary.Add(node.ID, node);
            }

            var material = new ConvectionDiffusionProperties(capacityCoeff: 0d, diffusionCoeff: DiffusionCoeff, convectionCoeff: ConvectionCoeff , dependentSourceCoeff: 0d, independentSourceCoeff: 0d);

            var elementFactory = new ConvectionDiffusionElement2DFactory(commonThickness: 1, material);

            var elementNodes = new IReadOnlyList<Node>[]
            {
                new List<Node>() { nodes[0], nodes[1], nodes[5], nodes[4] },
                new List<Node>() { nodes[1], nodes[2], nodes[6], nodes[5] },
                new List<Node>() { nodes[2], nodes[3], nodes[7], nodes[6] },
                new List<Node>() { nodes[4], nodes[5], nodes[9], nodes[8] },
                new List<Node>() { nodes[5], nodes[6], nodes[10], nodes[9] },
                new List<Node>() { nodes[6], nodes[7], nodes[11], nodes[10]},
                new List<Node>() { nodes[8], nodes[9], nodes[13], nodes[12]},
                new List<Node>() { nodes[9], nodes[10], nodes[14], nodes[13]},
                new List<Node>() { nodes[10], nodes[11], nodes[15], nodes[14]}

            };

            var elements = new ConvectionDiffusionElement2D[]
            {
                elementFactory.CreateElement(CellType.Quad4, elementNodes[0]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[1]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[2]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[3]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[4]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[5]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[6]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[7]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[8])
            };

            for (int i = 0; i < elements.Length; i++)
            {
                model.ElementsDictionary.Add(i, elements[i]);
                model.SubdomainsDictionary[0].Elements.Add(elements[i]);
            }

            model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
                new[]
                {          
                    new NodalUnknownVariable(nodes[0], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(nodes[4], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(nodes[8], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(nodes[12], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(nodes[3], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariable(nodes[7], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariable(nodes[11], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariable(nodes[15], ConvectionDiffusionDof.UnknownVariable, 50d)
                },
                new INodalConvectionDiffusionNeumannBoundaryCondition[]{}
            ));

            return model;
        }

        public static void CheckResults(double[] numericalSolution)
        {
            if (numericalSolution.Length != prescribedSolution.Length)
            {
                Console.WriteLine("Array Lengths do not match");
                return;
            }

            var isAMatch = true;
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                Console.WriteLine("Numerical: {0} \tPrescribed: {1}", numericalSolution[i], prescribedSolution[i]);
                if (Math.Abs((prescribedSolution[i] - numericalSolution[i]) / prescribedSolution[i]) > 1E-6)
                {
                    isAMatch = false;
                    // break;
                }
            }
            if (isAMatch == true)
            {
                Console.WriteLine("MSolve Solution matches prescribed solution");
                Console.WriteLine("Test Passed!");
            }
            else
            {
                Console.WriteLine("MSolve Solution does not match prescribed solution");
                Console.WriteLine("Test Failed!");
            }
        }
    }
}
