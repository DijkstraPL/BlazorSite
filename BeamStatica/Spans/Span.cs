using BeamStatica._spans.Interfaces;
using BeamStatica.Loads.ContinousLoads;
using BeamStatica.Loads.Interfaces;
using BeamStatica.Materials.Intefaces;
using BeamStatica.Nodes;
using BeamStatica.Nodes.Interfaces;
using BeamStatica.Sections.Interfaces;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using Tools;

namespace BeamStatica._spans
{
    public class Span : ISpan
    {
        public short Number { get; set; }

        public INode LeftNode { get; }

        [Abbreviation("L")]
        [Unit("m")]
        public double Length { get; }

        public INode RightNode { get; }

        public IYoungModulus Material { get; }

        public IMomentOfInteria Section { get; }

        public ICollection<ContinousLoad> ContinousLoads { get; set; }
        public ICollection<ILoad> PointLoads { get; set; }

        public StiffnessMatrix StiffnessMatrix { get; }
        public Vector<double> LoadVector { get; private set; }
        public Vector<double> Displacements { get; private set; }
        public Vector<double> Forces { get; private set; }

        public Span(Node leftNode, double length, INode rightNode, IYoungModulus material, IMomentOfInteria section)
        {
            LeftNode = leftNode ?? throw new ArgumentNullException(nameof(leftNode));
            Length = length;
            RightNode = rightNode ?? throw new ArgumentNullException(nameof(rightNode));
            Material = material ?? throw new ArgumentNullException(nameof(material));
            Section = section ?? throw new ArgumentNullException(nameof(section));

            ContinousLoads = new List<ContinousLoad>();
            PointLoads = new List<ILoad>();
            StiffnessMatrix = new StiffnessMatrix(this);
        }

        public void CalculateSpanLoadVector()
        {
            LoadVector = Vector<double>.Build.Dense(4);

            LoadVector[0] = ContinousLoads.Sum(cl => CalculateShear(cl, leftNode: true)) +
                PointLoads.Sum(pl => CalculateShearFromPointLoad(pl, leftNode: true));

            LoadVector[1] = ContinousLoads.Sum(cl => CalculateBendingMoment(cl, leftNode: true)) +
                PointLoads.Sum(pl => CalculateMomentFromPointLoad(pl, leftNode: true));

            LoadVector[2] = ContinousLoads.Sum(cl => CalculateShear(cl, leftNode: false)) +
               PointLoads.Sum(pl => CalculateShearFromPointLoad(pl, leftNode: false));

            LoadVector[3] = ContinousLoads.Sum(cl => CalculateBendingMoment(cl, leftNode: false)) +
                PointLoads.Sum(pl => CalculateMomentFromPointLoad(pl, leftNode: false));
        }

        public void CalculateDisplacement(Vector<double> deflectionVector, int numberOfDegreesOfFreedom)
        {
            Displacements = Vector<double>.Build.Dense(4);

            if (LeftNode.MovementNumber < numberOfDegreesOfFreedom)
                Displacements[0] = deflectionVector[LeftNode.MovementNumber];
            if (LeftNode.RotationNumber < numberOfDegreesOfFreedom)
                Displacements[1] = deflectionVector[LeftNode.RotationNumber];
            if (RightNode.MovementNumber < numberOfDegreesOfFreedom)
                Displacements[2] = deflectionVector[RightNode.MovementNumber];
            if (RightNode.RotationNumber < numberOfDegreesOfFreedom)
                Displacements[3] = deflectionVector[RightNode.RotationNumber];
        }
               
        public void SetDisplacement()
        {
            if (LeftNode.Rotation != null)
                LeftNode.Rotation.Value = Displacements[0];
            if (LeftNode.Deflection != null)
                LeftNode.Deflection.Value = Displacements[1];
            if (RightNode.Rotation != null)
                RightNode.Rotation.Value = Displacements[2];
            if (RightNode.Deflection != null)
                RightNode.Deflection.Value = Displacements[3];
        }

        public void CalculateForce()
        {
            Forces = StiffnessMatrix.Matrix.Multiply(Displacements).Add(LoadVector);
        }

        private double CalculateMomentFromPointLoad(ILoad pointLoad, bool leftNode)
        {
            double sign = leftNode ? 1.0 : -1.0;
            double distanceFromCloserNode = leftNode ? pointLoad.Position : Length - pointLoad.Position;
            double distanceFromOtherNode = leftNode ? Length - pointLoad.Position : pointLoad.Position;
            return sign * (-pointLoad.Value * distanceFromCloserNode * Math.Pow(distanceFromOtherNode, 2)) / Math.Pow(Length, 2);
        }

        private double CalculateShearFromPointLoad(ILoad pointLoad, bool leftNode)
        {
            double distanceFromCloserNode = leftNode ? pointLoad.Position : Length - pointLoad.Position;
            double distanceFromOtherNode = leftNode ? Length - pointLoad.Position : pointLoad.Position;
            return (-pointLoad.Value * Math.Pow(distanceFromOtherNode, 2) *
                (3 * distanceFromCloserNode + distanceFromOtherNode)) /
                Math.Pow(Length, 3);
        }

        private double CalculateBendingMoment(ContinousLoad continousLoad, bool leftNode)
        {
            double sign = leftNode ? 1.0 : -1.0;
            double closerLoad = leftNode ? -continousLoad.StartPosition.Value : -continousLoad.EndPosition.Value;
            double furtherLoad = leftNode ? -continousLoad.EndPosition.Value : -continousLoad.StartPosition.Value;
            double distanceFromCalculatedNode = leftNode ? continousLoad.StartPosition.Position : Length - continousLoad.EndPosition.Position;
            double loadLength = continousLoad.EndPosition.Position - continousLoad.StartPosition.Position;
            double distanceToOtherNode = leftNode ? Length - continousLoad.EndPosition.Position : continousLoad.StartPosition.Position;

            return sign / 60 * closerLoad * loadLength *
                   (3 * Math.Pow(loadLength, 3) +
                   15 * Math.Pow(loadLength, 2) * distanceFromCalculatedNode +
                   10 * Math.Pow(distanceToOtherNode, 2) * loadLength +
                   30 * Math.Pow(distanceToOtherNode, 2) * distanceFromCalculatedNode +
                   10 * Math.Pow(loadLength, 2) * distanceToOtherNode +
                   40 * distanceFromCalculatedNode * loadLength * distanceToOtherNode) /
                   Math.Pow(Length, 2) +
                   sign / 60 * furtherLoad * loadLength *
                   (2 * Math.Pow(loadLength, 3) +
                   5 * Math.Pow(loadLength, 2) * distanceFromCalculatedNode +
                   20 * Math.Pow(distanceToOtherNode, 2) * loadLength +
                   30 * Math.Pow(distanceToOtherNode, 2) * distanceFromCalculatedNode +
                   10 * Math.Pow(loadLength, 2) * distanceToOtherNode +
                   20 * distanceFromCalculatedNode * loadLength * distanceToOtherNode) /
                   Math.Pow(Length, 2);
        }

        private double CalculateShear(ContinousLoad continousLoad, bool leftNode)
        {
            double closerLoad = leftNode ? -continousLoad.StartPosition.Value : -continousLoad.EndPosition.Value;
            double furtherLoad = leftNode ? -continousLoad.EndPosition.Value : -continousLoad.StartPosition.Value;
            double distanceFromCalculatedNode = leftNode ? continousLoad.StartPosition.Position : Length - continousLoad.EndPosition.Position;
            double loadLength = continousLoad.EndPosition.Position - continousLoad.StartPosition.Position;
            double distanceToOtherNode = leftNode ? Length - continousLoad.EndPosition.Position : continousLoad.StartPosition.Position;

            return 1.0 / 20 * furtherLoad * loadLength *
                (3 * Math.Pow(loadLength, 3) +
                5 * Math.Pow(loadLength, 2) * distanceFromCalculatedNode +
                10 * Math.Pow(distanceToOtherNode, 3) +
                30 * Math.Pow(distanceToOtherNode, 2) * loadLength +
                30 * Math.Pow(distanceToOtherNode, 2) * distanceFromCalculatedNode +
                15 * Math.Pow(loadLength, 2) * distanceToOtherNode +
                20 * distanceFromCalculatedNode * loadLength * distanceToOtherNode) /
                Math.Pow(Length, 3) +
                1.0 / 20 * closerLoad * loadLength *
                (7 * Math.Pow(loadLength, 3) +
                15 * Math.Pow(loadLength, 2) * distanceFromCalculatedNode +
                10 * Math.Pow(distanceToOtherNode, 3) +
                 30 * Math.Pow(distanceToOtherNode, 2) * loadLength +
                 30 * Math.Pow(distanceToOtherNode, 2) * distanceFromCalculatedNode +
                 25 * Math.Pow(loadLength, 2) * distanceToOtherNode +
                 40 * distanceFromCalculatedNode * loadLength * distanceToOtherNode) /
                 Math.Pow(Length, 3);
        }
    }
}
