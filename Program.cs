using System;

namespace RicochetMono
{
  public static class Program
  {
    [STAThread]
    static void Main()
    {
      using (RicochetGame game = new RicochetGame())
      {
        game.Run();
      }
    }
  }
}
