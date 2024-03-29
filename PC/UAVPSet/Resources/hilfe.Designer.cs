﻿//------------------------------------------------------------------------------
// <auto-generated>
//     Dieser Code wurde von einem Tool generiert.
//     Laufzeitversion:2.0.50727.1433
//
//     Änderungen an dieser Datei können falsches Verhalten verursachen und gehen verloren, wenn
//     der Code erneut generiert wird.
// </auto-generated>
//------------------------------------------------------------------------------

namespace UAVP.UAVPSet.Resources {
    using System;
    
    
    /// <summary>
    ///   Eine stark typisierte Ressourcenklasse zum Suchen von lokalisierten Zeichenfolgen usw.
    /// </summary>
    // Diese Klasse wurde von der StronglyTypedResourceBuilder automatisch generiert
    // -Klasse über ein Tool wie ResGen oder Visual Studio automatisch generiert.
    // Um einen Member hinzuzufügen oder zu entfernen, bearbeiten Sie die .ResX-Datei und führen dann ResGen
    // mit der /str-Option erneut aus, oder Sie erstellen Ihr VS-Projekt neu.
    [global::System.CodeDom.Compiler.GeneratedCodeAttribute("System.Resources.Tools.StronglyTypedResourceBuilder", "2.0.0.0")]
    [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
    [global::System.Runtime.CompilerServices.CompilerGeneratedAttribute()]
    internal class hilfe {
        
        private static global::System.Resources.ResourceManager resourceMan;
        
        private static global::System.Globalization.CultureInfo resourceCulture;
        
        [global::System.Diagnostics.CodeAnalysis.SuppressMessageAttribute("Microsoft.Performance", "CA1811:AvoidUncalledPrivateCode")]
        internal hilfe() {
        }
        
        /// <summary>
        ///   Gibt die zwischengespeicherte ResourceManager-Instanz zurück, die von dieser Klasse verwendet wird.
        /// </summary>
        [global::System.ComponentModel.EditorBrowsableAttribute(global::System.ComponentModel.EditorBrowsableState.Advanced)]
        internal static global::System.Resources.ResourceManager ResourceManager {
            get {
                if (object.ReferenceEquals(resourceMan, null)) {
                    global::System.Resources.ResourceManager temp = new global::System.Resources.ResourceManager("UAVP.UAVPSet.Resources.hilfe", typeof(hilfe).Assembly);
                    resourceMan = temp;
                }
                return resourceMan;
            }
        }
        
        /// <summary>
        ///   Überschreibt die CurrentUICulture-Eigenschaft des aktuellen Threads für alle
        ///   Ressourcenzuordnungen, die diese stark typisierte Ressourcenklasse verwenden.
        /// </summary>
        [global::System.ComponentModel.EditorBrowsableAttribute(global::System.ComponentModel.EditorBrowsableState.Advanced)]
        internal static global::System.Globalization.CultureInfo Culture {
            get {
                return resourceCulture;
            }
            set {
                resourceCulture = value;
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Influence of the air pressure sensor on the gas (proportional): 1 ähnelt.
        /// </summary>
        internal static string baro {
            get {
                return ResourceManager.GetString("baro", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Influence of the air pressure sensor on the gas (differential): 4 ähnelt.
        /// </summary>
        internal static string baroDif {
            get {
                return ResourceManager.GetString("baroDif", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Temperature compensation for air pressure sensor (use = 13) ähnelt.
        /// </summary>
        internal static string baroTemp {
            get {
                return ResourceManager.GetString("baroTemp", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Value -99 bis 99 (mostly -20 to +20)
        ///Serves the attitude of the reactivity. Usually the differential value becomes negative set sign to proportional value
        ///The value is too high, when the UFO is bouncing up or lies unstably in the air.
        ///For YAW use 0. ähnelt.
        /// </summary>
        internal static string Differenzial {
            get {
                return ResourceManager.GetString("Differenzial", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Not supported yet!!! ähnelt.
        /// </summary>
        internal static string EbenenAusgleich {
            get {
                return ResourceManager.GetString("EbenenAusgleich", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Mostly positive
        ///Indicates which receiver impulses being evaluated (positive/negatively). ähnelt.
        /// </summary>
        internal static string EmpfaengerImpulse {
            get {
                return ResourceManager.GetString("EmpfaengerImpulse", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Graupner-modus (K1=Gas, K2=Roll, K3=Nick, K4=Gier)
        ///Futaba/Robbe-modus (K1=Roll, K2=Nick, K3=Gas, K4=Gier) ähnelt.
        /// </summary>
        internal static string GasKanal {
            get {
                return ResourceManager.GetString("GasKanal", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Roll- und Nick-Steuerknüppelsignale werden in der Wirkung halbiert (Specky-bit)
        ///This is particularly  for proportional values &gt; 20 and helpful for beginners! ähnelt.
        /// </summary>
        internal static string HalbRollNick {
            get {
                return ResourceManager.GetString("HalbRollNick", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Updaterate of controllers
        ///Min  value +3
        ///Engines have to start faultless! ähnelt.
        /// </summary>
        internal static string Impuls {
            get {
                return ResourceManager.GetString("Impuls", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die values -99 bis 99 (same Vorzeichen like proportional)
        ///The control portion definiting the total gyro value over time. Important for the Heading Lock of the yaw channel. A too high value permits the UFO bouncing up, a too small value doesn´t turn back  the UFO exactly after an angular momentum to its initial position. ähnelt.
        /// </summary>
        internal static string Integral {
            get {
                return ResourceManager.GetString("Integral", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Integral Limiter 
        ///Always positiv! 
        ///Maximum 127/Integral=Integral Limiter
        ///(Example Integral= 6  --&gt;  127/6=21.16 rounded offt=21, Integral Limiter may be Max 21!) ähnelt.
        /// </summary>
        internal static string IntegralLimiter {
            get {
                return ResourceManager.GetString("IntegralLimiter", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die LEDs showing condition of integral (only indicating function, no influence on the parameters) ähnelt.
        /// </summary>
        internal static string Integrierzustand {
            get {
                return ResourceManager.GetString("Integrierzustand", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Indicates how strong the pitch/rolling motions at the camera will become balanced (Tilt). ähnelt.
        /// </summary>
        internal static string Kameraausgleich {
            get {
                return ResourceManager.GetString("Kameraausgleich", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Defines how heavy the deviation of target direction  intervenes
        ///in the regulation. Values about -99 to +99, default is 0 ähnelt.
        /// </summary>
        internal static string Kompass {
            get {
                return ResourceManager.GetString("Kompass", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die The Engines have to run after for some seconds, when gas is at zero!  ähnelt.
        /// </summary>
        internal static string Leerlaufgas {
            get {
                return ResourceManager.GetString("Leerlaufgas", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Limiter (always positive !)
        ///Since 3.13 absolent for Roll and Nick. Use 25 for YAW ähnelt.
        /// </summary>
        internal static string Limiter {
            get {
                return ResourceManager.GetString("Limiter", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Neutral value of the linear sensor for this axle when exact Neutral value of the linear sensor for this axle, with accurately horizontal situation of the UAVP.
        ///Value investigated in linearsensor, when accu sticked - must be transferred over the function! ähnelt.
        /// </summary>
        internal static string Neutral {
            get {
                return ResourceManager.GetString("Neutral", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Value -99 to 99 
        ///The controls interest,which is measured and/or through handed directy by transmitter and gyros.
        ///The value is too small, when the UFO reacts badly to roll/nodding to your instructions. The value is too high, if the UFO is bouncing up.
        ///For YAW use 0. ähnelt.
        /// </summary>
        internal static string Proportional {
            get {
                return ResourceManager.GetString("Proportional", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Shows on with which tension  the piezo summer/red LED responds.
        ///Avoids over-discharging of the accus and crashes  by missing thrust performance.
        ///Values by 3s from 43 to 45 ähnelt.
        /// </summary>
        internal static string Unterspannung {
            get {
                return ResourceManager.GetString("Unterspannung", resourceCulture);
            }
        }
        
        /// <summary>
        ///   Sucht eine lokalisierte Zeichenfolge, die Shows teh way to fly.
        ///+ = an arm forward into flight direction
        ///X = 2 arms in iflight driection (45° staggerd to  + modus)
        ///Note: X mode requires other parameters to prop. Diff.  integral ähnelt.
        /// </summary>
        internal static string XModus {
            get {
                return ResourceManager.GetString("XModus", resourceCulture);
            }
        }
    }
}
