using BeamStatica.Loads.ContinousLoads;
using BeamStatica.Loads.PointLoads;
using BeamStatica.Materials;
using BeamStatica.Nodes;
using BeamStatica.Sections;
using BeamStatica._spans;
using NUnit.Framework;
using System.Linq;

namespace BeamStatica.Tests
{
    [TestFixture()]
    public class BeamTests
    {
        [Test()]
        public void FullBeamCalculationTest1()
        {
            var material = new Material() { YoungModulus = 30 };
            var section1 = new RectangleSection(width: 300, height: 500);
            var section2 = new RectangleSection(width: 200, height: 300);

            var node1 = new FixedNode();
            var node2 = new FreeNode();
            var node3 = new SupportedNode();
            var node4 = new FixedNode();

            node2.ConcentratedForces.Add(new ShearLoad(-200));
            node2.ConcentratedForces.Add(new BendingMoment(150));

            var nodes = new Node[] { node1, node2, node3, node4 };

            var span1 = new Span(
                leftNode: node1,
                length: 6,
                rightNode: node2,
                material: material,
                section: section1
                );

            var span2 = new Span(
                leftNode: node2,
                length: 4,
                rightNode: node3,
                material: material,
                section: section2
                );

            var span3 = new Span(
                leftNode: node3,
                length: 10,
                rightNode: node4,
                material: material,
                section: section2
                );

            var spans = new Span[] { span1, span2, span3 };

            var startLoad = new ShearLoad(0, 0);
            var endLoad = new ShearLoad(-30, 2.5);
            var startLoad2 = new ShearLoad(-30, 2.5);
            var endLoad2 = new ShearLoad(-30, 7.5);
            var startLoad3 = new ShearLoad(-30, 7.5);
            var endLoad3 = new ShearLoad(0, 10);
            var pointLoad = new ShearLoad(-150, 3);

            span3.ContinousLoads.Add(new ContinousLoad(startLoad, endLoad));
            span3.ContinousLoads.Add(new ContinousLoad(startLoad2, endLoad2));
            span3.ContinousLoads.Add(new ContinousLoad(startLoad3, endLoad3));
            span1.PointLoads.Add(pointLoad);

            var beam = new Beam(spans, nodes);

            beam.Calculate();

            Assert.That(node1.ShearForce.Value, Is.EqualTo(208.87).Within(0.01));
            Assert.That(node1.BendingMoment.Value, Is.EqualTo(-632.29).Within(0.01));

            Assert.That(node3.ShearForce.Value, Is.EqualTo(256.78).Within(0.01));

            Assert.That(node4.ShearForce.Value, Is.EqualTo(109.35).Within(0.01));
            Assert.That(node4.BendingMoment.Value, Is.EqualTo(212.17).Within(0.01));
        }

        [Test()]
        public void FullBeamCalculationTest2()
        {
            var material = new Material() { YoungModulus = 30 };
            var section1 = new RectangleSection(width: 300, height: 500);
            var section2 = new RectangleSection(width: 200, height: 300);

            var node1 = new FixedNode();
            var node2 = new FreeNode();
            var node3 = new SupportedNode();
            var node4 = new SupportedNode();

            node2.ConcentratedForces.Add(new ShearLoad(-200));
            node3.ConcentratedForces.Add(new BendingMoment(90));

            var nodes = new Node[] { node1, node2, node3, node4 };

            var span1 = new Span(
                leftNode: node1,
                length: 6,
                rightNode: node2,
                material: material,
                section: section1
                );

            var span2 = new Span(
                leftNode: node2,
                length: 4,
                rightNode: node3,
                material: material,
                section: section2
                );

            var span3 = new Span(
                leftNode: node3,
                length: 10,
                rightNode: node4,
                material: material,
                section: section2
                );

            var spans = new Span[] { span1, span2, span3 };

            var startLoad = new ShearLoad(-30, 0);
            var endLoad = new ShearLoad(0, 6);
            var pointLoad = new ShearLoad(-150, 5);

            span1.ContinousLoads.Add(new ContinousLoad(startLoad, endLoad));
            span3.PointLoads.Add(pointLoad);

            var beam = new Beam(spans, nodes);

            beam.Calculate();

            Assert.That(node1.ShearForce.Value, Is.EqualTo(155.95).Within(0.01));
            Assert.That(node1.BendingMoment.Value, Is.EqualTo(-352.22).Within(0.01));

            Assert.That(node3.ShearForce.Value, Is.EqualTo(231.32).Within(0.01));

            Assert.That(node4.ShearForce.Value, Is.EqualTo(52.73).Within(0.01));
        }

        [Test()]
        public void FullBeamCalculationTest3()
        {
            var material = new Material() { YoungModulus = 30 };
            var section1 = new RectangleSection(width: 300, height: 600);

            var node1 = new FreeNode();
            var node2 = new SupportedNode();
            var node3 = new SupportedNode();
            var node4 = new FixedNode();

            node1.ConcentratedForces.Add(new ShearLoad(-200));
            node3.ConcentratedForces.Add(new BendingMoment(90));

            var nodes = new Node[] { node1, node2, node3, node4 };

            var span1 = new Span(
                leftNode: node1,
                length: 3,
                rightNode: node2,
                material: material,
                section: section1
                );

            var span2 = new Span(
                leftNode: node2,
                length: 4,
                rightNode: node3,
                material: material,
                section: section1
                );

            var span3 = new Span(
                leftNode: node3,
                length: 10,
                rightNode: node4,
                material: material,
                section: section1
                );

            var spans = new Span[] { span1, span2, span3 };

            var startLoad = new ShearLoad(-30, 2);
            var endLoad = new ShearLoad(-30, 6);

            span3.ContinousLoads.Add(new ContinousLoad(startLoad, endLoad));

            var beam = new Beam(spans, nodes);

            beam.Calculate();

            Assert.That(node2.ShearForce.Value, Is.EqualTo(335.33).Within(0.01));

            Assert.That(node3.ShearForce.Value, Is.EqualTo(-87.22).Within(0.01));

            Assert.That(node4.ShearForce.Value, Is.EqualTo(71.90).Within(0.01));
            Assert.That(node4.BendingMoment.Value, Is.EqualTo(207.65).Within(0.01));
        }

    }
}