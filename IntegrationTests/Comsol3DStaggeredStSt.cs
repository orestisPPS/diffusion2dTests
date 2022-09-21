using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Isoparametric;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;
using MGroup.FEM.Helpers;

namespace ConvectionDiffusionTest
{
    public static class Comsol3DStaggeredStSt
	{
        //public static double[] ConvectionCoeff => new[]  {1d, 1d, 1d};
		//public static double DiffusionCoeff => 1d;// 2d;
        //public static double DependentProductionCoeff => 1d;
		//public static double IndependentProductionCoeff => 1d;// 2d;

        public static double[] prescribedSolution = new double[] { 113.24999999999996 }; // [1, 1, 1] node id 13



		public static Model CreateModelFromComsolFile(string filename, double[] ConvectionCoeff, double DiffusionCoeff, double DependentProductionCoeff, double IndependentProductionCoeff, double Capacity = 0d)
        {
			var model = new Model();
			model.SubdomainsDictionary[0] = new Subdomain(id: 0);
			
			var reader = new ComsolMeshReader(filename);

            foreach (var node in reader.NodesDictionary.Values)
            {
                model.NodesDictionary.Add(node.ID, node);
            }

			var material = new ConvectionDiffusionProperties(
				capacityCoeff: Capacity,
				diffusionCoeff: DiffusionCoeff,
				convectionCoeff: ConvectionCoeff,
				dependentSourceCoeff: DependentProductionCoeff,
				independentSourceCoeff: IndependentProductionCoeff);

			var elementFactory = new ConvectionDiffusionElement3DFactory(material);

			foreach (var elementConnectivity in reader.ElementConnectivity)
			{
				var element = elementFactory.CreateElement(elementConnectivity.Value.Item1, elementConnectivity.Value.Item2);
				model.ElementsDictionary.Add(elementConnectivity.Key, element);
				model.SubdomainsDictionary[0].Elements.Add(element);
			}

			var topNodes = new List<INode>();
            var bottomNodes = new List<INode>();
            
            foreach (var node in model.NodesDictionary.Values)
            {
                if (Math.Abs(2 - node.Y) < 1E-9) topNodes.Add(node);
                if (Math.Abs(0 - node.Y) < 1E-9) bottomNodes.Add(node);
            }

			int i = 0;
            var dirichletBCs = new NodalUnknownVariable[topNodes.Count + bottomNodes.Count];
			foreach (var node in topNodes)
			{
				dirichletBCs[i] = new NodalUnknownVariable(node, ConvectionDiffusionDof.UnknownVariable, 100d);
				i++;
			}
			foreach (var node in bottomNodes)
			{
				dirichletBCs[i] = new NodalUnknownVariable(node, ConvectionDiffusionDof.UnknownVariable, 50d);
				i++;
			}

			model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
				dirichletBCs,
				new INodalConvectionDiffusionNeumannBoundaryCondition[] { }
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
				var error = Math.Abs((prescribedSolution[i] - numericalSolution[i]) / prescribedSolution[i]);

				Console.WriteLine("Numerical: {0} \tPrescribed: {1} \t AbsoluteRelativeError: {2}", numericalSolution[i], prescribedSolution[i], error);
                if (error > 1E-6)
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
