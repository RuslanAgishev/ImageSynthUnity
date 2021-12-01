using UnityEngine;
using System.Collections;
using System.Collections.Generic;
//using UnityEditor;
/// @cond
public class ITSDirectionReference : MonoBehaviour {
	
//	Vector3 size = Vector3.zero;
//	float timer = 0;
	
void OnDrawGizmos()
	{
		Gizmos.color = Color.yellow;
		Gizmos.DrawLine(transform.position + transform.forward , transform.position + transform.forward + transform.right * 2);
		Gizmos.DrawLine(transform.position + transform.forward * 2 , transform.position + transform.forward * 2 + transform.right * 2);
		Gizmos.DrawLine(transform.position + transform.forward * 3 ,transform.position + transform.forward * 3 + transform.right * 2);
		Gizmos.DrawLine(transform.position + transform.forward * 4 ,transform.position + transform.forward * 4 + transform.right * 2);
		Gizmos.DrawLine(transform.position + transform.forward * 5 ,transform.position + transform.forward * 5 + transform.right * 2);
		Gizmos.DrawLine(transform.position + transform.forward * 6 ,transform.position + transform.forward * 6 + transform.right * 2);
		Gizmos.DrawLine(transform.position - transform.forward ,transform.position - transform.forward + transform.right * 2);
		Gizmos.DrawLine(transform.position - transform.forward * 2 ,transform.position - transform.forward * 2 + transform.right * 2);
		Gizmos.DrawLine(transform.position - transform.forward * 3 , transform.position - transform.forward * 3 + transform.right * 2);
		Gizmos.DrawLine(transform.position - transform.forward * 4 , transform.position - transform.forward * 4 + transform.right * 2);
		Gizmos.DrawLine(transform.position - transform.forward * 5 , transform.position - transform.forward * 5 + transform.right * 2);
		Gizmos.DrawLine(transform.position - transform.forward * 6 , transform.position - transform.forward * 6 + transform.right * 2);
		
//		if (size.magnitude == 0 || Time.realtimeSinceStartup - timer > 5)
//		{
//			timer = Time.realtimeSinceStartup;
//			size = carSize();
//		}
		
			
//		IRDSTextGizmo.Draw(transform.position+ transform.right * 3 - transform.up * 0.25f,"Car total Length:  "+size.z);
//		IRDSTextGizmo.Draw(transform.position+ transform.right * 3 - transform.up * 0.5f,"Car total Width:   "+size.x);
//		IRDSTextGizmo.Draw(transform.position+ transform.right * 3 - transform.up * 0.75f,"Car total Height:  "+size.y);
//		
//		IRDSTextGizmo.Draw(transform.position+ transform.right * 2,"0mts");
//		IRDSTextGizmo.Draw(transform.position - transform.forward * 1+ transform.right * 2,"1Mts");
//		IRDSTextGizmo.Draw(transform.position - transform.forward * 2+ transform.right * 2,"2Mts");
//		IRDSTextGizmo.Draw(transform.position - transform.forward * 3+ transform.right * 2,"3Mts");
//		IRDSTextGizmo.Draw(transform.position - transform.forward * 4+ transform.right * 2,"4Mts");
//		IRDSTextGizmo.Draw(transform.position - transform.forward * 5+ transform.right * 2,"5Mts");
//		IRDSTextGizmo.Draw(transform.position - transform.forward * 6+ transform.right * 2,"6Mts");
//		
//		IRDSTextGizmo.Draw(transform.position + transform.forward * 1+ transform.right * 2,"1Mts");
//		IRDSTextGizmo.Draw(transform.position + transform.forward * 2+ transform.right * 2,"2Mts");
//		IRDSTextGizmo.Draw(transform.position + transform.forward * 3+ transform.right * 2,"3Mts");
//		IRDSTextGizmo.Draw(transform.position + transform.forward * 4+ transform.right * 2,"4Mts");
//		IRDSTextGizmo.Draw(transform.position + transform.forward * 5+ transform.right * 2,"5Mts");
//		IRDSTextGizmo.Draw(transform.position + transform.forward * 6+ transform.right * 2,"6Mts");
		
		
		
		Gizmos.color = Color.blue;
		Gizmos.DrawLine(transform.position - transform.forward * 2 , transform.forward * 2);
		Gizmos.DrawLine(transform.forward * 2, -transform.up * 2 + transform.forward * 2);
		Gizmos.DrawLine(-transform.up * 2 + transform.forward * 2, transform.forward * 5f + transform.up);
		Gizmos.DrawLine(transform.forward * 5f + transform.up , transform.up * 4 + transform.forward * 2);
		Gizmos.DrawLine(transform.up * 4 + transform.forward * 2, transform.up * 2 + transform.forward * 2);
		Gizmos.DrawLine(transform.up * 2 + transform.forward * 2,transform.position+ transform.up * 2- transform.forward * 2);
		Gizmos.DrawLine(transform.position+ transform.up * 2- transform.forward * 2, transform.position - transform.forward * 2);
		
		Gizmos.color = Color.red;
		Gizmos.DrawLine(transform.position, transform.right * 15);
		Gizmos.DrawLine(transform.position, -transform.right * 15);
		
		Gizmos.color = Color.green;
		Gizmos.DrawLine(transform.position, transform.up * 15);
		Gizmos.DrawLine(transform.position, -transform.up * 15);
		
		Gizmos.color = Color.blue;
		Gizmos.DrawLine(transform.position, transform.forward * 15);
		Gizmos.DrawLine(transform.position, -transform.forward * 15);
		
	}
	
	
	
	
	
//public class IRDSTextGizmo
//
//	{
//	
//	    private static IRDSTextGizmo tg = null;
//	
//	    private Dictionary<char,string> texturePathLookup;
//	
//	    private Camera editorCamera = null;
//	
//	    private const int CHAR_TEXTURE_HEIGHT = 8; // todo: line breaks
//	
//	    private const int CHAR_TEXTURE_WIDTH = 7;
//	
//	    private const string characters = "abcdefghijklmnopqrstuvwxyz0123456789:., ";
//	
//	    
//	
//	    
//	
//	    public static void Init()
//	
//	    {
//	
//	        tg = new IRDSTextGizmo();
//	
//	    }
//	
//	    
//	
//	    
//	
//	    /* singleton constructor */
//	
//	    private IRDSTextGizmo()
//	
//	    {
//	
//	        editorCamera = Camera.current;
//	
//	        texturePathLookup = new Dictionary<char,string>();
//	
//	        for( int c=0; c<characters.Length; c++ ){
//	
//				if (characters[c] == ':')
//					texturePathLookup.Add( characters[c], "/TextIcons/text_colon.png" );
//				else
//	            	texturePathLookup.Add( characters[c], "/TextIcons/text_" + characters[c] + ".png" );
//	
//	        }
//	
//	    }
//	
//	    
//	
//	    
//	
//	    /* only call this method from a OnGizmos() method */
//	
//	    public static void Draw( Vector3 position, string text )
//	
//	    {
//	
//	        if( tg == null ) Init();
//	
//	        
//	
//	        string lowerText = text.ToLower();
//	
//	        Vector3 screenPoint = tg.editorCamera.WorldToScreenPoint( position );
//	
//	        int offset = 0;
//	
//	        for( int c=0; c<lowerText.Length; c++ )
//	
//	        {   
//	
//	            if( tg.texturePathLookup.ContainsKey( lowerText[c] ) )
//	
//	            {
//	
//	                Vector3 worldPoint = tg.editorCamera.ScreenToWorldPoint( new Vector3( screenPoint.x + offset, screenPoint.y, screenPoint.z ) );
//	
//	                
//					char a = lowerText[c];
//	                Gizmos.DrawIcon( worldPoint, tg.texturePathLookup[ a ],true );
//	
//	                
//	
//	                offset += CHAR_TEXTURE_WIDTH + 2;
//	
//	            }
//	
//	        }
//	
//	    }
//	
//	}
	
}
/// @endcond