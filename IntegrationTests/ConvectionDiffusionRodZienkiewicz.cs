using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Line;
using MGroup.MSolve.Discretization.Entities;
using MGroup.NumericalAnalyzers.Logging;

namespace ConvectionDiffusionTest
{
    public static class ConvectionDiffusionRodZienkiewicz
    {
        public static double[] ConvectionCoeff => new[]  {1d};
        public static double DiffusionCoeff => 1d;

        public static double Length => 1d;

        public static Model CreateModel()
        {
            var model = new Model();
            model.SubdomainsDictionary.Add(0, new Subdomain(0));
            var nodes = new Node[]
            {
                new Node(id : 0, x : 0d,   y : 0d),
                new Node(id : 1, x : 1E-1, y : 0d),
                new Node(id : 2, x : 2E-1, y : 0d),
                new Node(id : 3, x : 3E-1, y : 0d),
                new Node(id : 4, x : 4E-1, y : 0d),
                new Node(id : 5, x : 5E-1, y : 0d),
                new Node(id : 6, x : 6E-1, y : 0d),
                new Node(id : 7, x : 7E-1, y : 0d),
                new Node(id : 8, x : 8E-1, y : 0d),
                new Node(id : 9, x : 9E-1, y : 0d),
                new Node(id : 10, x : 1E-0, y : 0d)
            };
            foreach (var node in nodes)
            {
                model.NodesDictionary.Add(node.ID, node);
            }
            
            var material = new ConvectionDiffusionProperties(capacityCoeff : 0d, diffusionCoeff : DiffusionCoeff, convectionCoeff : ConvectionCoeff, dependentSourceCoeff : 0d, independentSourceCoeff : 0d);

            var elements = new ConvectionDiffusionRod[]
            {
                new ConvectionDiffusionRod(new [] {nodes[0], nodes[1]}, crossSectionArea : 1d, material),
                new ConvectionDiffusionRod(new [] {nodes[1], nodes[2]}, crossSectionArea : 1d, material),
                new ConvectionDiffusionRod(new [] {nodes[2], nodes[3]}, crossSectionArea : 1d, material),
                new ConvectionDiffusionRod(new [] {nodes[3], nodes[4]}, crossSectionArea : 1d, material),
                new ConvectionDiffusionRod(new [] {nodes[4], nodes[5]}, crossSectionArea : 1d, material),
                new ConvectionDiffusionRod(new [] {nodes[5], nodes[6]}, crossSectionArea : 1d, material),
                new ConvectionDiffusionRod(new [] {nodes[6], nodes[7]}, crossSectionArea : 1d, material),
                new ConvectionDiffusionRod(new [] {nodes[7], nodes[8]}, crossSectionArea : 1d, material),
                new ConvectionDiffusionRod(new [] {nodes[8], nodes[9]}, crossSectionArea : 1d, material),
                new ConvectionDiffusionRod(new [] {nodes[9], nodes[10]}, crossSectionArea : 1d, material)
            };


            //foreach (var element in elements)
            //{
                //model.ElementsDictionary.Add(element.ID, element);
                //model.SubdomainsDictionary[0].Elements.Add(element);
            //}
            for (int i = 0; i <elements.Length; i++)
            {
                model.ElementsDictionary.Add(i, elements[i]);
                model.SubdomainsDictionary[0].Elements.Add(elements[i]);
            }
            model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
                new []
                {
                    new NodalUnknownVariable(nodes[0],  ConvectionDiffusionDof.UnknownVariable, 1d),
                    new NodalUnknownVariable(nodes[10], ConvectionDiffusionDof.UnknownVariable, 0d)     
                },
                new INodalConvectionDiffusionNeumannBoundaryCondition[] {} 
            ));
            return model;
        }

        public static Func<double, double> rodAnalyticalSolution = (x) => (Math.Exp(ConvectionCoeff[0] * x / DiffusionCoeff) - Math.Exp(ConvectionCoeff[0] * Length / DiffusionCoeff))
                                                                          / (1d - Math.Exp(ConvectionCoeff[0] * Length / DiffusionCoeff));
        public static double[] CalculateAnalyticalSolution()
        {
            var solution = new double[9];
            for (int i = 0; i < solution.Length; i++)
            {
                var x = 0.1 * (i + 1);
                solution[i] = rodAnalyticalSolution(x);
            }
            return solution;
        }
        public static void CheckResults(double[] numericalSolution)
        {
            var prescribedSolution = CalculateAnalyticalSolution();
            if (numericalSolution.Length != prescribedSolution.Length)
            {
                Console.WriteLine("Array Lengths do not match");
                return;
            }

            var isAMatch = true;
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                if (Math.Abs((prescribedSolution[i] - numericalSolution[i]) / prescribedSolution[i]) > 1E-6)
                {
                    isAMatch = false;
                    break;
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
