using BeamStatica.Materials.Intefaces;
using BeamStatica.Sections.Interfaces;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace BeamStatica._spans.Interfaces
{
    public interface ISpan : ILengthProvider
    {
        IMomentOfInteria Section { get; }
        IYoungModulus Material { get; }
    }
}
