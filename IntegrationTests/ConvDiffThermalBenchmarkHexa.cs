using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Isoparametric;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;
using MGroup.FEM.Helpers;

namespace ConvectionDiffusionTest
{
    public static class ConvectionDiffusionThermalBenchmarkHexa
    {   
        public static double[] ConvectionCoeff => new[]  {0d, 0d, 0d};
        public static double DiffusionCoeff => 1d;
        
        public static double[] prescribedSolution = new double[] { 135.054, 158.824, 135.054, 469.004, 147.059, 159.327, 178.178, 147.299, 139.469, 147.059, 191.717, 147.059, 135.054, 158.824, 135.054, 469.004, 147.059, 159.327 };
			
        public static Model CreateModel()
        {
            var model = new Model();
			model.SubdomainsDictionary[0] = new Subdomain(id: 0);

			int nodeIndex = -1;
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 0.0);

			var elementNodes = new INode[][]
			{
				new[] { model.NodesDictionary[13], model.NodesDictionary[4], model.NodesDictionary[3], model.NodesDictionary[12], model.NodesDictionary[10], model.NodesDictionary[1], model.NodesDictionary[0], model.NodesDictionary[9] },
				new[] { model.NodesDictionary[14], model.NodesDictionary[5], model.NodesDictionary[4], model.NodesDictionary[13], model.NodesDictionary[11], model.NodesDictionary[2], model.NodesDictionary[1], model.NodesDictionary[10] },
				new[] { model.NodesDictionary[16], model.NodesDictionary[7], model.NodesDictionary[6], model.NodesDictionary[15], model.NodesDictionary[13], model.NodesDictionary[4], model.NodesDictionary[3], model.NodesDictionary[12] },
				new[] { model.NodesDictionary[17], model.NodesDictionary[8], model.NodesDictionary[7], model.NodesDictionary[16], model.NodesDictionary[14], model.NodesDictionary[5], model.NodesDictionary[4], model.NodesDictionary[13] },
				new[] { model.NodesDictionary[22], model.NodesDictionary[13], model.NodesDictionary[12], model.NodesDictionary[21], model.NodesDictionary[19], model.NodesDictionary[10], model.NodesDictionary[9], model.NodesDictionary[18] },
				new[] { model.NodesDictionary[23], model.NodesDictionary[14], model.NodesDictionary[13], model.NodesDictionary[22], model.NodesDictionary[20], model.NodesDictionary[11], model.NodesDictionary[10], model.NodesDictionary[19] },
				new[] { model.NodesDictionary[25], model.NodesDictionary[16], model.NodesDictionary[15], model.NodesDictionary[24], model.NodesDictionary[22], model.NodesDictionary[13], model.NodesDictionary[12], model.NodesDictionary[21] },
				new[] { model.NodesDictionary[26], model.NodesDictionary[17], model.NodesDictionary[16], model.NodesDictionary[25], model.NodesDictionary[23], model.NodesDictionary[14], model.NodesDictionary[13], model.NodesDictionary[22] },
			};
			// // int nodeIndex = -1;
			// model.NodesDictionary[0] = new Node(id: 0, x: 0d, y: 0d, z: 0d);
			// model.NodesDictionary[1] = new Node(id: 1, x: 1.5d, y: 0d, z: 0d);
			// model.NodesDictionary[2] = new Node(id: 2, x: 0d, y: 1.5d, z: 0d);
			// model.NodesDictionary[3] = new Node(id: 3, x: 0d, y: 0d, z: 1.5d);
			// model.NodesDictionary[4] = new Node(id: 4, x: 3d, y: 0d, z: 0d);
			// model.NodesDictionary[5] = new Node(id: 5, x: 1.5d, y: 1.5d, z: 0d);
			// model.NodesDictionary[6] = new Node(id: 6, x: 1.5d, y: 0d, z: 1.5d);
			// model.NodesDictionary[7] = new Node(id: 7, x: 0d, y: 3d, z: 0d);
			// model.NodesDictionary[8] = new Node(id: 8, x: 0d, y: 1.5d, z: 1.5d);
			// model.NodesDictionary[9] = new Node(id: 9, x: 0d, y: 0d, z: 3d);
			// model.NodesDictionary[10] = new Node(id: 10, x: 3d, y: 1.5d, z: 0d);
			// model.NodesDictionary[11] = new Node(id: 11, x: 3d, y: 0d, z: 1.5d);
			// model.NodesDictionary[12] = new Node(id: 12, x: 1.5d, y: 3d, z: 0d);
			// model.NodesDictionary[13] = new Node(id: 13, x: 1.5d, y: 1.5d, z: 1.5d);
			// model.NodesDictionary[14] = new Node(id: 14, x: 1.5d, y: 0d, z: 3d);
			// model.NodesDictionary[15] = new Node(id: 15, x: 0d, y: 3d, z: 1.5d);
			// model.NodesDictionary[16] = new Node(id: 16, x: 0d, y: 1.5d, z: 3d);
			// model.NodesDictionary[17] = new Node(id: 17, x: 3d, y: 3d, z: 0d);
			// model.NodesDictionary[18] = new Node(id: 18, x: 3d, y: 1.5d, z: 1.5d);
			// model.NodesDictionary[19] = new Node(id: 19, x: 3d, y: 0d, z: 3d);
			// model.NodesDictionary[20] = new Node(id: 20, x: 1.5d, y: 3d, z: 1.5d);
			// model.NodesDictionary[21] = new Node(id: 21, x: 1.5d, y: 1.5d, z: 3d);
			// model.NodesDictionary[22] = new Node(id: 22, x: 0d, y: 3d, z: 3d);
			// model.NodesDictionary[23] = new Node(id: 23, x: 3d, y: 3d, z: 1.5d);
			// model.NodesDictionary[24] = new Node(id: 24, x: 3d, y: 1.5d, z: 3d);
			// model.NodesDictionary[25] = new Node(id: 25, x: 1.5d, y: 3d, z: 3d);
			// model.NodesDictionary[26] = new Node(id: 26, x: 3d, y: 3d, z: 3d);

			// var elementNodes = new INode[][]
			// {
			// 	new[] { model.NodesDictionary[0], model.NodesDictionary[1], model.NodesDictionary[2], model.NodesDictionary[5], model.NodesDictionary[3], model.NodesDictionary[6], model.NodesDictionary[8], model.NodesDictionary[13] },
			// 	new[] { model.NodesDictionary[1], model.NodesDictionary[4], model.NodesDictionary[5], model.NodesDictionary[10], model.NodesDictionary[6], model.NodesDictionary[11], model.NodesDictionary[13], model.NodesDictionary[18] },
			// 	new[] { model.NodesDictionary[2], model.NodesDictionary[5], model.NodesDictionary[7], model.NodesDictionary[12], model.NodesDictionary[8], model.NodesDictionary[13], model.NodesDictionary[15], model.NodesDictionary[20] },
			// 	new[] { model.NodesDictionary[3], model.NodesDictionary[6], model.NodesDictionary[8], model.NodesDictionary[13], model.NodesDictionary[9], model.NodesDictionary[14], model.NodesDictionary[16], model.NodesDictionary[21] },
			// 	new[] { model.NodesDictionary[5], model.NodesDictionary[10], model.NodesDictionary[12], model.NodesDictionary[17], model.NodesDictionary[13], model.NodesDictionary[18], model.NodesDictionary[20], model.NodesDictionary[23] },
			// 	new[] { model.NodesDictionary[6], model.NodesDictionary[11], model.NodesDictionary[13], model.NodesDictionary[18], model.NodesDictionary[14], model.NodesDictionary[19], model.NodesDictionary[21], model.NodesDictionary[24] },
			// 	new[] { model.NodesDictionary[8], model.NodesDictionary[13], model.NodesDictionary[15], model.NodesDictionary[20], model.NodesDictionary[16], model.NodesDictionary[21], model.NodesDictionary[22], model.NodesDictionary[25] },
			// 	new[] { model.NodesDictionary[13], model.NodesDictionary[18], model.NodesDictionary[20], model.NodesDictionary[23], model.NodesDictionary[21], model.NodesDictionary[24], model.NodesDictionary[25], model.NodesDictionary[26] },
			// };
			var nodeReordering = new GMeshElementLocalNodeOrdering();
			var rearrangeNodes = elementNodes.Select(x => nodeReordering.ReorderNodes(x, CellType.Hexa8)).ToArray();

            var material = new ConvectionDiffusionProperties(capacityCoeff: 0d, diffusionCoeff: DiffusionCoeff, convectionCoeff: ConvectionCoeff , dependentSourceCoeff: 0d, independentSourceCoeff: 0d);

			var elementFactory = new ConvectionDiffusionElement3DFactory(material);
            for (int i = 0; i < elementNodes.Length; i++)
			{
				model.ElementsDictionary[i] = elementFactory.CreateElement(CellType.Hexa8, rearrangeNodes[i]);
				model.ElementsDictionary[i].ID = i;
				model.SubdomainsDictionary[0].Elements.Add(model.ElementsDictionary[i]);
			}
;
			model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
				new[]
				{
					new NodalUnknownVariable(model.NodesDictionary[0], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[1], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[2], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[4], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[5], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[7], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[10], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[12], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[17], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[9], ConvectionDiffusionDof.UnknownVariable,  100d),
					new NodalUnknownVariable(model.NodesDictionary[14], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[16], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[19], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[21], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[22], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[24], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[25], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[26], ConvectionDiffusionDof.UnknownVariable, 100d),
				},
                new INodalConvectionDiffusionNeumannBoundaryCondition[] {} 
			));

			return model;
        }

        public static void CheckResults(double[] numericalSolution)
        {
            // if (numericalSolution.Length != prescribedSolution.Length)
            // {
            //     Console.WriteLine("Array Lengths do not match");
            //     return;
            // }

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
