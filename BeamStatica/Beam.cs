using BeamStatica.Loads.PointLoads;
using BeamStatica.Nodes.Interfaces;
using BeamStatica.Results.Interfaces;
using BeamStatica.Results.OnSpan;
using BeamStatica._spans;
using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;

namespace BeamStatica
{

    public class Beam
    {
        public IGetResult ShearResult { get; }
        public IGetResult BendingMomentResult { get; }
        public IGetResult RotationResult { get; }
        public IGetResult DeflectionResult { get; }

        public short NumberOfDegreesOfFreedom { get; private set; }

        public IList<Span> Spans { get; }
        public ICollection<INode> Nodes { get; }

        public GlobalStiffnessMatrix GlobalStiffnessMatrix { get; }

        public Vector<double> JointLoadVector { get; private set; }
        public Vector<double> SpanLoadVector { get; private set; }

        public Vector<double> DeflectionVector { get; private set; }

        public Beam(IList<Span> spans, ICollection<INode> nodes)
        {
            Spans = spans ?? throw new ArgumentNullException(nameof(spans));
            Nodes = nodes ?? throw new ArgumentNullException(nameof(nodes));

            GlobalStiffnessMatrix = new GlobalStiffnessMatrix(this);
            ShearResult = new ShearResult(Spans);
            BendingMomentResult = new BendingMomentResult(Spans);
            RotationResult = new RotationResult(Spans);
            DeflectionResult = new DeflectionResult(Spans);
        }

        public void Calculate()
        {
            SetNumeration();
            CalculateStiffnessMatrixes();
            GlobalStiffnessMatrix.Calculate();
            CaluclateJointLoadVector();
            CalculateSpanLoadVectors();
            CalculateSpanLoadVector();
            CalculateDeflectionVector();
            CalculateDisplacements();
            CalculateForces();
            CalculateReactions();
        }


        private void SetNumeration()
        {
            short spanCounter = 0;
            short nodeCounter = 0;

            spanCounter = SetSpanNumeration(spanCounter);
            nodeCounter = SetNodeNumeration(nodeCounter);

            SetNumberOfDegreesOfFreedom();
        }

        private void CalculateStiffnessMatrixes()
        {
            foreach (var span in Spans)
                span.StiffnessMatrix.Calculate();
        }

        private void CaluclateJointLoadVector()
        {
            JointLoadVector = Vector<double>.Build.Dense(NumberOfDegreesOfFreedom);

            for (int i = 0; i < NumberOfDegreesOfFreedom; i++)
            {
                if (Nodes.Any(n => n.MovementNumber == i))
                    JointLoadVector[i] = Nodes.SingleOrDefault(n => n.MovementNumber == i)?.ConcentratedForces.Sum(cf => (cf as ShearLoad)?.Value ?? 0) ?? 0;
                else
                    JointLoadVector[i] = Nodes.SingleOrDefault(n => n.RotationNumber == i)?.ConcentratedForces.Sum(cf => -(cf as BendingMoment)?.Value ?? 0) ?? 0;
            }
        }

        private void CalculateSpanLoadVectors()
        {
            foreach (var span in Spans)
                span.CalculateSpanLoadVector();
        }

        private void CalculateSpanLoadVector()
        {
            SpanLoadVector = Vector<double>.Build.Dense(NumberOfDegreesOfFreedom);
            foreach (var span in Spans)
                CalculateSpanLoadVectorForCurrentSpan(span);
        }

        private void CalculateDeflectionVector()
        {
            DeflectionVector = GlobalStiffnessMatrix.InversedMatrix * (JointLoadVector - SpanLoadVector);
        }

        private void CalculateDisplacements()
        {
            foreach (var span in Spans)
            {
                span.CalculateDisplacement(DeflectionVector, NumberOfDegreesOfFreedom);
                span.SetDisplacement();
            }
        }

        private void CalculateForces()
        {
            foreach (var span in Spans)
                span.CalculateForce();
        }

        private void CalculateReactions()
        {
            int numberOfReactions = Spans.Count * 2 + 2 - NumberOfDegreesOfFreedom;

            for (int i = NumberOfDegreesOfFreedom; i < numberOfReactions + NumberOfDegreesOfFreedom; i++)
            {
                var shearForce = Spans.SingleOrDefault(s => s.LeftNode.MovementNumber == i)?.LeftNode.ShearForce;
                if (shearForce != null)
                    shearForce.Value += Spans.Where(s => s.LeftNode.MovementNumber == i).Sum(s => s.Forces[0]);

                if (Spans.SingleOrDefault(s => s.LeftNode.RotationNumber == i)?.LeftNode.BendingMoment != null)
                    Spans.SingleOrDefault(s => s.LeftNode.RotationNumber == i).LeftNode.BendingMoment.Value -= Spans.Where(s => s.LeftNode.RotationNumber == i).Sum(s => s.Forces[1]);

                if (Spans.SingleOrDefault(s => s.RightNode.MovementNumber == i)?.RightNode.ShearForce != null)
                    Spans.SingleOrDefault(s => s.RightNode.MovementNumber == i).RightNode.ShearForce.Value += Spans.Where(s => s.RightNode.MovementNumber == i).Sum(s => s.Forces[2]);

                if (Spans.SingleOrDefault(s => s.RightNode.RotationNumber == i)?.RightNode.BendingMoment != null)
                    Spans.SingleOrDefault(s => s.RightNode.RotationNumber == i).RightNode.BendingMoment.Value -= Spans.Where(s => s.RightNode.RotationNumber == i).Sum(s => s.Forces[3]);
            }
        }

        private short SetSpanNumeration(short spanCounter)
        {
            foreach (var span in Spans)
                span.Number = spanCounter++;
            return spanCounter;
        }

        private short SetNodeNumeration(short nodeCounter)
        {
            foreach (var node in Nodes)
                node.SetDisplacementNumeration(ref nodeCounter);
            foreach (var node in Nodes)
                node.SetReactionNumeration(ref nodeCounter);
            return nodeCounter;
        }

        private void SetNumberOfDegreesOfFreedom()
        {
            foreach (var node in Nodes)
                NumberOfDegreesOfFreedom += node.DegreesOfFreedom;
        }

        private void CalculateSpanLoadVectorForCurrentSpan(Span span)
        {
            for (int i = 0; i < NumberOfDegreesOfFreedom; i++)
            {
                if (span.LeftNode.MovementNumber == i)
                    SpanLoadVector[i] += span.LoadVector[0];
                else if (span.LeftNode.RotationNumber == i)
                    SpanLoadVector[i] += span.LoadVector[1];
                else if (span.RightNode.MovementNumber == i)
                    SpanLoadVector[i] += span.LoadVector[2];
                else if (span.RightNode.RotationNumber == i)
                    SpanLoadVector[i] += span.LoadVector[3];
            }
        }
    }
}
